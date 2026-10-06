# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Read-only depth-to-RGB reprojection. Never assumes a depth for missing pixels."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from dimos.hardware.manipulators.ur7e.policy import BridgeError


def transform_matrix(translation: list[float], quaternion_xyzw: list[float]) -> list[list[float]]:
    q = np.asarray(quaternion_xyzw, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise BridgeError("INVALID_TRANSFORM", "Invalid transform quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    t = np.asarray(translation, dtype=float)
    if t.shape != (3,) or not np.all(np.isfinite(t)):
        raise BridgeError("INVALID_TRANSFORM", "Invalid transform translation")
    matrix[:3, 3] = t
    return matrix.tolist()


def roi_points(
    depth: np.ndarray[Any, np.dtype[Any]],
    depth_info: dict[str, Any],
    color_info: dict[str, Any],
    color_from_depth: list[list[float]],
    roi: list[int],
    depth_unit_m: float,
) -> np.ndarray[Any, np.dtype[Any]]:
    """Reproject real depth to RGB; return the measured color-optical points inside the ROI."""
    if len(roi) != 4 or any(isinstance(v, bool) or not isinstance(v, int) for v in roi):
        raise BridgeError("INVALID_ROI", "ROI must be four integer pixel coordinates")
    left, top, right, bottom = roi
    if (
        not 0 <= left < right <= color_info["width"]
        or not 0 <= top < bottom <= color_info["height"]
    ):
        raise BridgeError("INVALID_ROI", "ROI must be inside the RGB image")
    if depth.dtype != np.uint16 or depth.shape != (depth_info["height"], depth_info["width"]):
        raise BridgeError("DEPTH_ENCODING", "Depth shape/type does not match camera intrinsics")
    kd = np.asarray(depth_info["K"], dtype=float).reshape(3, 3)
    kc = np.asarray(color_info["K"], dtype=float).reshape(3, 3)
    if any(not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0 for k in (kd, kc)):
        raise BridgeError("INVALID_INTRINSICS", "Camera focal lengths must be finite and positive")
    transform = np.asarray(color_from_depth, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise BridgeError("INVALID_TRANSFORM", "Depth-to-color extrinsics are unavailable")
    if (
        not np.allclose(transform[3], [0, 0, 0, 1])
        or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-5)
    ):
        raise BridgeError("INVALID_TRANSFORM", "Extrinsics must be a proper rigid transform")
    if not np.isfinite(depth_unit_m) or not 0 < depth_unit_m <= 0.01:
        raise BridgeError("DEPTH_SCALE", "Depth unit must be verified before localization")
    rows, cols = np.nonzero((depth > 0) & (depth < 65535))
    z = depth[rows, cols].astype(float) * depth_unit_m
    pixels = np.column_stack((cols, rows)).astype(float).reshape(-1, 1, 2)
    if not len(pixels):
        raise BridgeError("NO_TARGET_DEPTH", "No valid depth samples")
    rays = cv2.undistortPoints(pixels, kd, np.asarray(depth_info["D"], dtype=float)).reshape(-1, 2)
    points = np.column_stack((rays * z[:, None], z))
    points = points @ transform[:3, :3].T + transform[:3, 3]
    points = points[points[:, 2] > 0]
    if not len(points):
        raise BridgeError("NO_TARGET_DEPTH", "No depth samples in front of the RGB camera")
    pixels, _ = cv2.projectPoints(
        points, np.zeros(3), np.zeros(3), kc, np.asarray(color_info["D"], dtype=float)
    )
    uv = pixels.reshape(-1, 2)
    selected = points[
        (uv[:, 0] >= left) & (uv[:, 0] < right) & (uv[:, 1] >= top) & (uv[:, 1] < bottom)
    ]
    if len(selected) < 10:
        raise BridgeError("NO_TARGET_DEPTH", "ROI has fewer than ten measured depth samples")
    return np.asarray(selected, dtype=float)


def locate_rgb_roi(
    depth: np.ndarray[Any, np.dtype[Any]],
    depth_info: dict[str, Any],
    color_info: dict[str, Any],
    color_from_depth: list[list[float]],
    roi: list[int],
    depth_unit_m: float,
) -> dict[str, Any]:
    """Reproject real depth to RGB; return a robust visible-surface point, not a grasp pose."""
    selected = roi_points(depth, depth_info, color_info, color_from_depth, roi, depth_unit_m)
    q25, median, q75 = np.percentile(selected[:, 2], [25, 50, 75])
    spread = float(q75 - q25)
    if spread > 0.06:
        raise BridgeError(
            "AMBIGUOUS_TARGET_DEPTH", "ROI mixes surfaces; select a smaller object patch"
        )
    selected = selected[np.abs(selected[:, 2] - median) <= max(0.01, spread * 2)]
    return {
        "point_color_optical_m": np.median(selected, axis=0).tolist(),
        "measured_depth_samples": len(selected),
        "depth_iqr_m": spread,
        "roi_rgb": roi,
        "surface_point_not_object_center": True,
        "base_frame_available": False,
        "executed": False,
    }


def top_surface_point(
    points: np.ndarray[Any, np.dtype[Any]],
    base_from_camera: np.ndarray[Any, np.dtype[Any]],
    min_height_m: float = 0.01,
    band_m: float = 0.02,
) -> dict[str, Any]:
    """Top face of an object seen at an angle: keep the highest points above the table.

    `points` are color-optical points inside the object's image box; the box also
    catches side faces and table, so only the band just below the highest robust
    height (in the arm base frame, table at z~0) is kept.
    """
    base = points @ base_from_camera[:3, :3].T + base_from_camera[:3, 3]
    above = base[base[:, 2] > min_height_m]
    if len(above) < 20:
        raise BridgeError("NO_TARGET_DEPTH", "Too few depth samples above the table in the box")
    top = float(np.percentile(above[:, 2], 95))
    face = above[above[:, 2] >= top - band_m]
    if len(face) < 10:
        raise BridgeError("NO_TARGET_DEPTH", "Top face has too few depth samples")
    centre = np.median(face, axis=0)
    camera_from_base = np.linalg.inv(base_from_camera)
    return {
        "point_base_link_m": centre.tolist(),
        "point_color_optical_m": (
            camera_from_base[:3, :3] @ centre + camera_from_base[:3, 3]
        ).tolist(),
        "top_height_m": float(centre[2]),
        "top_face_samples": len(face),
        "samples_above_table": len(above),
    }
