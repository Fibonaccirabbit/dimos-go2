# Copyright 2026 Fibonaccirabbit
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

"""Capture live evidence from the native apartment stack; never feed scene truth to the Agent."""

import argparse
from collections import Counter
import json
from pathlib import Path
import time

import lcm
import numpy as np
from PIL import Image as PILImage

from dimos.core.global_config import global_config
from dimos.core.rpc_client import RPCClient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.simulation.dimsim.scene_client import SceneClient


def capture(output: Path, seconds: float, queries: list[str]) -> None:
    global_config.update(transport="lcm")
    bus = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    counts: Counter[str] = Counter()
    latest = {}
    poses = []
    types = {
        "color_image": Image,
        "odom": PoseStamped,
        "lidar": PointCloud2,
        "global_map": PointCloud2,
        "global_costmap": OccupancyGrid,
    }

    def receive(channel: str, data: bytes) -> None:
        name = channel.split("#")[0].lstrip("/")
        msg = types[name].lcm_decode(data)
        counts[name] += 1
        latest[name] = msg
        if name == "odom":
            poses.append({"ts": msg.ts, "x": msg.x, "y": msg.y, "z": msg.z})

    subs = [bus.subscribe(f"/{name}#{typ.msg_name}", receive) for name, typ in types.items()]
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            bus.handle_timeout(100)
    finally:
        for sub in subs:
            bus.unsubscribe(sub)
    missing = set(types) - set(latest)
    if missing:
        raise RuntimeError(f"Missing streams: {sorted(missing)}; counts={dict(counts)}")

    output.mkdir(parents=True, exist_ok=True)
    frame = latest["color_image"]
    assert frame.save(str(output / "camera.jpg"))
    grid = latest["global_costmap"]
    colors = np.zeros((*grid.grid.shape, 3), dtype=np.uint8)
    colors[grid.grid == 0] = [160, 180, 210]
    colors[grid.grid > 0] = [255, 90, 90]
    PILImage.fromarray(np.flipud(colors)).save(output / "costmap.png")
    summary = {
        "seconds": seconds,
        "counts": dict(counts),
        "rgb_hz": counts["color_image"] / seconds,
        "camera_shape": frame.shape,
        "frame_age_s": time.time() - frame.ts,
        "lidar_points": len(latest["lidar"]),
        "global_map_points": len(latest["global_map"]),
        "costmap": {
            "shape": grid.grid.shape,
            "resolution": grid.resolution,
            "origin": {"x": grid.origin.x, "y": grid.origin.y},
            "free_cells": int(np.count_nonzero(grid.grid == 0)),
            "occupied_cells": int(np.count_nonzero(grid.grid > 0)),
            "unknown_cells": int(np.count_nonzero(grid.grid < 0)),
        },
        "poses": poses,
    }
    memory = RPCClient.remote(SpatialMemory)
    navigation = RPCClient.remote(ReplanningAStarPlanner)
    try:
        summary["memory_stats"] = memory.get_stats()
        summary["memory_queries"] = {q: memory.query_by_text(q, limit=3) for q in queries}
        summary["navigation"] = {
            "state": str(navigation.get_state()),
            "goal_reached": navigation.is_goal_reached(),
        }
        summary["memory_saved"] = memory.save()
    finally:
        memory.stop_rpc_client()
        navigation.stop_rpc_client()
    scene = SceneClient()
    scene.start()
    try:
        summary["evaluator_only_scene"] = scene.exec("return {agentPosition: agent.getPosition()};")
    finally:
        scene.stop()
    (output / "evidence.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    printable = {k: v for k, v in summary.items() if k not in {"poses", "memory_queries"}}
    printable["last_pose"] = poses[-1]
    print(json.dumps(printable, ensure_ascii=False, default=str))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--query", action="append", default=[])
    args = parser.parse_args()
    capture(args.output, args.seconds, args.query)


if __name__ == "__main__":
    main()
