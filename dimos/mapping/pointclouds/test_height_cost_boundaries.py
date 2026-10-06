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

import numpy as np
import pytest

from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.planning_preview import cell_cost


@pytest.mark.parametrize("height", [0.0, 0.8, -0.8])
def test_missing_diagonal_is_unknown_not_an_origin_dependent_false_slope(height):
    points = np.array(
        [[x * 0.05, y * 0.05, height] for x in range(3) for y in range(3) if (x, y) != (2, 2)]
    )
    grid = height_cost_occupancy(PointCloud2.from_numpy(points), smoothing=0.0)
    assert cell_cost(grid, 0.075, 0.075) == -1
    assert cell_cost(grid, 0.125, 0.125) == -1


@pytest.mark.parametrize("height", [0.0, 0.8, -0.8])
def test_observed_flat_ground_remains_free_at_any_world_height(height):
    points = np.array([[x * 0.05, y * 0.05, height] for x in range(7) for y in range(7)])
    grid = height_cost_occupancy(PointCloud2.from_numpy(points), smoothing=0.0)
    assert cell_cost(grid, 0.175, 0.175) == 0
    assert cell_cost(grid, 0.7, 0.7) == -1


@pytest.mark.parametrize("height", [0.0, 0.8, -0.8])
def test_real_height_step_is_not_erased_by_boundary_fix(height):
    points = np.array(
        [[x * 0.05, y * 0.05, height + (0.4 if x >= 4 else 0)] for x in range(9) for y in range(9)]
    )
    grid = height_cost_occupancy(PointCloud2.from_numpy(points), smoothing=0.0)
    # Sample cell interiors, not floating-point-sensitive grid boundaries.
    assert cell_cost(grid, 0.175, 0.225) == 100
    assert cell_cost(grid, 0.225, 0.225) == 100
    assert cell_cost(grid, 0.125, 0.225) == 0
