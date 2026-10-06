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

from typing import Any

import numpy as np
import pytest

from dimos.hardware.manipulators.ur7e.policy import BridgeError
from dimos.hardware.manipulators.ur7e.target_geometry import (
    locate_rgb_roi,
    top_surface_point,
    transform_matrix,
)


@pytest.fixture
def intrinsics() -> dict[str, Any]:
    return {"width": 12, "height": 12, "K": [10, 0, 5, 0, 10, 5, 0, 0, 1], "D": [0.0] * 5}


def test_reprojects_depth_with_real_extrinsic_translation(intrinsics: dict[str, Any]) -> None:
    depth = np.full((12, 12), 1000, np.uint16)
    transform = transform_matrix([0.2, 0, 0], [0, 0, 0, 1])
    result = locate_rgb_roi(depth, intrinsics, intrinsics, transform, [6, 4, 10, 8], 0.001)
    assert result["point_color_optical_m"] == pytest.approx([0.25, 0.05, 1.0])
    assert result["measured_depth_samples"] == 16
    assert result["base_frame_available"] is False
    assert result["executed"] is False


@pytest.mark.parametrize("value", [0, 65535])
def test_never_invents_depth_for_invalid_pixels(intrinsics: dict[str, Any], value: int) -> None:
    with pytest.raises(BridgeError, match="No valid"):
        locate_rgb_roi(
            np.full((12, 12), value, np.uint16),
            intrinsics,
            intrinsics,
            np.eye(4).tolist(),
            [0, 0, 12, 12],
            0.001,
        )


def test_mixed_background_is_rejected(intrinsics: dict[str, Any]) -> None:
    depth = np.full((12, 12), 1000, np.uint16)
    depth[6:] = 2000
    with pytest.raises(BridgeError, match="mixes surfaces"):
        locate_rgb_roi(depth, intrinsics, intrinsics, np.eye(4).tolist(), [0, 0, 12, 12], 0.001)


@pytest.mark.parametrize("roi", [[-1, 0, 5, 5], [3, 3, 3, 8], [0, 0, 13, 12], [True, 0, 5, 5]])
def test_invalid_roi_is_rejected(intrinsics: dict[str, Any], roi: list[int]) -> None:
    with pytest.raises(BridgeError):
        locate_rgb_roi(
            np.full((12, 12), 1000, np.uint16),
            intrinsics,
            intrinsics,
            np.eye(4).tolist(),
            roi,
            0.001,
        )


def test_rotation_transform_direction() -> None:
    transform = np.array(transform_matrix([1, 2, 3], [0, 0, 1, 0]))
    assert transform @ [1, 0, 0, 1] == pytest.approx([0, 2, 3, 1])


def test_reflection_is_not_a_rigid_camera_rotation(intrinsics: dict[str, Any]) -> None:
    transform = np.eye(4)
    transform[0, 0] = -1
    with pytest.raises(BridgeError, match="proper rigid"):
        locate_rgb_roi(
            np.full((12, 12), 1000, np.uint16),
            intrinsics,
            intrinsics,
            transform.tolist(),
            [0, 0, 12, 12],
            0.001,
        )


def test_top_surface_ignores_side_face_and_table() -> None:
    rng = np.random.default_rng(0)
    xy = rng.uniform(-0.07, 0.07, size=(400, 2))
    top = np.column_stack((xy + np.array([0.5, 0.2]), np.full(400, 0.15)))
    side = np.column_stack(
        (np.full(300, 0.43), rng.uniform(0.13, 0.27, 300), rng.uniform(0.0, 0.15, 300))
    )
    table = np.column_stack((rng.uniform(0.3, 0.7, (300, 2)), np.zeros(300)))
    result = top_surface_point(np.vstack((top, side, table)), np.eye(4))
    assert result["top_height_m"] == pytest.approx(0.15, abs=0.005)
    assert result["point_base_link_m"][:2] == pytest.approx([0.5, 0.2], abs=0.01)
    assert result["point_color_optical_m"] == pytest.approx(result["point_base_link_m"])
