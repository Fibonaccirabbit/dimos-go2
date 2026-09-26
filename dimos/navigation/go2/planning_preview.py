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
"""Native dimOS costmap/A* preview with no velocity or navigation commands."""

from itertools import pairwise
import math
from threading import RLock
from time import monotonic
from typing import Any

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.go2.replanning_a_star.min_cost_astar import min_cost_astar


class PlanningPreviewConfig(ModuleConfig):
    max_age: float = Field(default=5.0, gt=0)
    footprint_width: float = Field(default=0.5, gt=0)
    max_relative_distance: float = Field(default=3.0, gt=0)


def cell_cost(grid: OccupancyGrid, x: float, y: float) -> int | None:
    point = grid.world_to_grid((x, y))
    if not 0 <= point.x < grid.width or not 0 <= point.y < grid.height:
        return None
    gx, gy = int(point.x), int(point.y)
    if not 0 <= gx < grid.width or not 0 <= gy < grid.height:
        return None
    return int(grid.grid[gy, gx])


def known_navigation_map(grid: OccupancyGrid, width: float) -> OccupancyGrid:
    # Unknown space is also inflated: the entire conservative footprint must
    # remain within observed traversable space, not merely the robot's center.
    strict = OccupancyGrid(
        grid=np.where(grid.grid < 0, 100, grid.grid).astype(np.int8),
        resolution=grid.resolution,
        origin=grid.origin,
        frame_id=grid.frame_id,
        ts=grid.ts,
    )
    return make_navigation_map(strict, width, "simple", "voronoi")


class Go2PlanningPreview(Module):
    config: PlanningPreviewConfig
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    path: Out[Path]
    navigation_costmap: Out[OccupancyGrid]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._pose: PoseStamped | None = None
        self._map: OccupancyGrid | None = None
        self._pose_received = 0.0
        self._map_received = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_pose)))
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_map)))

    def _on_pose(self, pose: PoseStamped) -> None:
        with self._lock:
            self._pose = pose
            self._pose_received = monotonic()

    def _on_map(self, grid: OccupancyGrid) -> None:
        with self._lock:
            self._map = grid
            self._map_received = monotonic()

    @skill
    def navigation_preview_status(self) -> dict[str, Any]:
        """Inspect real costmap and odometry. Navigation is preview-only; no movement."""
        with self._lock:
            pose, grid = self._pose, self._map
            result: dict[str, Any] = {
                "motion_enabled": False,
                "planner": "dimOS native min_cost_astar",
                "unknown_space_blocked": True,
                "footprint_width": self.config.footprint_width,
                "pose_age": None if pose is None else monotonic() - self._pose_received,
                "map_age": None if grid is None else monotonic() - self._map_received,
            }
            if pose is not None:
                result["robot_xy"] = [pose.x, pose.y]
                result["yaw"] = pose.yaw
            if grid is not None:
                result.update(
                    {
                        "frame": grid.frame_id,
                        "size": [grid.width, grid.height],
                        "resolution": grid.resolution,
                        "known_cells": int((grid.grid >= 0).sum()),
                        "free_cells": int((grid.grid == 0).sum()),
                        "occupied_cells": int((grid.grid >= 100).sum()),
                    }
                )
                if pose is not None:
                    result["robot_raw_cell_cost"] = cell_cost(grid, pose.x, pose.y)
                    nav = known_navigation_map(grid, self.config.footprint_width)
                    result["robot_inflated_cell_cost"] = cell_cost(nav, pose.x, pose.y)
            return result

    @skill
    def preview_relative_path(self, forward: float, left: float = 0.0) -> SkillResult:
        """Preview a path from actual robot pose to a relative metric goal.

        forward/left are meters in the robot's horizontal body frame. This only
        displays a path; it CANNOT walk, rotate, arrive, explore or execute a route.
        Rejects stale/missing data, frame mismatch, unknown and blocked space.
        """
        with self._lock:
            self.path.publish(Path())  # Clear a previous preview even on rejection.
            if not all(math.isfinite(value) for value in (forward, left)):
                return SkillResult.fail("INVALID_INPUT", "Distances must be finite")
            if not 0.05 <= math.hypot(forward, left) <= self.config.max_relative_distance:
                return SkillResult.fail(
                    "INVALID_INPUT",
                    f"Goal must be 0.05-{self.config.max_relative_distance} meters away",
                )
            pose, grid = self._pose, self._map
            if pose is None or grid is None:
                return SkillResult.fail("INVALID_STATE", "Waiting for real odometry and costmap")
            now = monotonic()
            if max(now - self._pose_received, now - self._map_received) > self.config.max_age:
                return SkillResult.fail("INVALID_STATE", "Real odometry or costmap is stale")
            if pose.frame_id != grid.frame_id:
                return SkillResult.fail("INVALID_STATE", "Pose and map frames do not match")
            goal = (
                pose.x + math.cos(pose.yaw) * forward - math.sin(pose.yaw) * left,
                pose.y + math.sin(pose.yaw) * forward + math.cos(pose.yaw) * left,
            )
            nav = known_navigation_map(grid, self.config.footprint_width)
            self.navigation_costmap.publish(nav)
            for label, point in (("start", (pose.x, pose.y)), ("goal", goal)):
                cost = cell_cost(nav, *point)
                if cost is None or not 0 <= cost < 100:
                    return SkillResult.fail(
                        "INVALID_STATE",
                        f"{label} footprint is outside observed traversable space; "
                        "do not clear unknown cells or assume a safe starting disc",
                    )
            route = min_cost_astar(nav, goal, (pose.x, pose.y), unknown_penalty=1.0, use_cpp=False)
            if not route:
                return SkillResult.fail(
                    "EXECUTION_FAILED", "No path within observed traversable space"
                )
            # Native A* is 8-connected. Reject diagonal corner cutting rather than
            # presenting a route through the corners of occupied/unknown cells.
            cells = [nav.world_to_grid((point.x, point.y)) for point in route.poses]
            for a, b in pairwise(cells):
                ax, ay, bx, by = int(a.x), int(a.y), int(b.x), int(b.y)
                if ax != bx and ay != by and max(nav.grid[ay, bx], nav.grid[by, ax]) >= 100:
                    return SkillResult.fail(
                        "EXECUTION_FAILED", "A* preview cuts a blocked diagonal corner"
                    )
            self.path.publish(route)
            length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in pairwise(route.poses))
            return SkillResult.ok(
                "Path displayed only. No robot motion was sent; arrival is unverified.",
                goal_xy=list(goal),
                waypoints=len(route.poses),
                length_meters=length,
                motion_enabled=False,
                frame=grid.frame_id,
            )
