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

"""Render an offline tip-path audit onto the matching captured image. No motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", required=True)
    parser.add_argument("--calibration-candidates", required=True)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    sample = np.load(args.sample)
    candidates = json.loads(Path(args.calibration_candidates).read_text())
    plan = json.loads(Path(args.plan).read_text())
    if plan["executed"] or plan["motion_allowed"] or not plan["plan_success"]:
        raise ValueError("Expected a successful offline-only plan")
    camera_from_base = np.asarray(
        candidates["candidates"][0]["color_optical_from_base_link"], dtype=float
    )
    path = np.asarray(plan["trajectory_gripper_tcp_base_link_m"], dtype=float)
    surface = np.asarray(plan["target_surface_base_link_m"], dtype=float)
    hover = np.asarray(plan["hover_gripper_tcp_base_link_m"], dtype=float)
    if not np.allclose(sample["joint_positions_rad"], plan["start_joint_positions_rad"], atol=1e-5):
        raise ValueError("Plan start does not match captured image configuration")
    points = np.concatenate([path, surface[None], hover[None]])
    camera_points = points @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
    if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
        raise ValueError("Invalid points for camera projection")
    pixels, _ = cv2.projectPoints(
        camera_points, np.zeros(3), np.zeros(3), sample["color_K"], sample["color_D"]
    )
    uv: NDArray[np.int32] = np.rint(pixels.reshape(-1, 2)).astype(np.int32)
    picture = sample["color_bgr"].copy()
    cyan, green, magenta = (255, 220, 0), (50, 255, 80), (255, 60, 255)
    cv2.polylines(picture, [uv[:-2].reshape(-1, 1, 2)], False, cyan, 2, cv2.LINE_AA)
    cv2.line(picture, tuple(uv[-2]), tuple(uv[-1]), magenta, 2, cv2.LINE_AA)
    cv2.circle(picture, tuple(uv[-2]), 6, green, 2, cv2.LINE_AA)
    cv2.circle(picture, tuple(uv[-1]), 6, magenta, 2, cv2.LINE_AA)
    cv2.putText(
        picture,
        "box surface",
        tuple(uv[-2] + [9, 14]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        green,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        picture,
        "tip +10cm",
        tuple(uv[-1] + [9, -4]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        magenta,
        1,
        cv2.LINE_AA,
    )
    cv2.rectangle(picture, (0, 0), (picture.shape[1], 49), (25, 25, 25), -1)
    cv2.putText(
        picture,
        "OFFLINE PREVIEW - NOT EXECUTED",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        picture,
        "Unvalidated camera fit / partial obstacles / tool offset",
        (10, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (150, 190, 255),
        1,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(args.output, picture):
        raise OSError(f"Could not save preview: {args.output}")
    print(json.dumps({"output": args.output, "executed": False, "trajectory_samples": len(path)}))


if __name__ == "__main__":
    main()
