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

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.go2 import planning_preview as mod
from dimos.navigation.go2.planning_preview import Go2PlanningPreview, cell_cost


@pytest.fixture
def preview(mocker):
    clock = mocker.patch.object(mod, "monotonic", return_value=100.0)
    planner = Go2PlanningPreview(g=GlobalConfig(), footprint_width=0.2)
    path_output = mocker.patch.object(planner.path, "publish")
    mocker.patch.object(planner.navigation_costmap, "publish")
    planner._on_pose(PoseStamped(position=[1, 1, 0], frame_id="world"))
    planner._on_map(OccupancyGrid(np.zeros((30, 30), dtype=np.int8), resolution=0.1))
    yield planner, path_output, clock
    planner.stop()


def test_native_astar_preview_displays_path_without_motion_capability(preview):
    planner, output, _ = preview
    result = planner.preview_relative_path(1.0)
    assert result.success is True
    assert result.metadata["motion_enabled"] is False
    assert result.metadata["goal_xy"] == [2.0, 1.0]
    route = output.call_args.args[0]
    assert len(route.poses) == 11
    assert (route.poses[0].x, route.poses[0].y) == (1.0, 1.0)
    assert (route.poses[-1].x, route.poses[-1].y) == (2.0, 1.0)
    assert not hasattr(planner, "cmd_vel") and not hasattr(planner, "nav_cmd_vel")


def test_unknown_start_is_not_carved_free(preview):
    planner, output, _ = preview
    grid = np.zeros((30, 30), dtype=np.int8)
    grid[10, 10] = -1
    planner._on_map(OccupancyGrid(grid, resolution=0.1))
    result = planner.preview_relative_path(1.0)
    assert result.success is False
    assert "start footprint" in result.message
    assert len(output.call_args.args[0].poses) == 0


def test_unknown_barrier_cannot_be_crossed(preview):
    planner, _, _ = preview
    grid = np.zeros((30, 30), dtype=np.int8)
    grid[:, 15] = -1
    planner._on_map(OccupancyGrid(grid, resolution=0.1))
    result = planner.preview_relative_path(1.0)
    assert result.success is False
    assert result.error_code == "EXECUTION_FAILED"


def test_stale_data_clears_previous_preview(preview):
    planner, output, clock = preview
    clock.return_value = 110.0
    result = planner.preview_relative_path(1.0)
    assert result.success is False
    assert "stale" in result.message
    assert len(output.call_args.args[0].poses) == 0


def test_frame_mismatch_rejected(preview):
    planner, _, _ = preview
    planner._on_pose(PoseStamped(position=[1, 1, 0], frame_id="base_link"))
    result = planner.preview_relative_path(1.0)
    assert result.success is False
    assert "frames do not match" in result.message


def test_relative_goal_uses_real_robot_yaw(preview):
    planner, _, _ = preview
    planner._on_pose(PoseStamped(position=[1, 1, 0], orientation=[0, 0, 1, 0], frame_id="world"))
    result = planner.preview_relative_path(0.5, 0.2)
    assert result.success is True
    assert result.metadata["goal_xy"] == pytest.approx([0.5, 0.8])


def test_outside_lower_map_edge_is_not_truncated_to_cell_zero():
    grid = OccupancyGrid(np.zeros((2, 2), dtype=np.int8), resolution=0.1)
    assert cell_cost(grid, -0.01, 0.0) is None
