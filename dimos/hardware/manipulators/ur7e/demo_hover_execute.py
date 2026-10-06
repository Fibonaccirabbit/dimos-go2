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

"""Workstation-only: execute a cuRobo hover plan via the scaled trajectory controller.

Operator-authorized demo. Re-times the plan to a joint speed cap, refuses to start
unless the live state matches the plan start and the robot is stationary, and
cancels the goal on Ctrl-C. Not a hardware E-stop.
"""

import argparse
from itertools import pairwise
import json
import math
from pathlib import Path
import time
from typing import Any

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def _duration(msg: Any, seconds: float) -> None:
    msg.sec = int(seconds)
    msg.nanosec = round((seconds - int(seconds)) * 1_000_000_000)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True)
    parser.add_argument("--controller", default="scaled_joint_trajectory_controller")
    parser.add_argument("--max-joint-speed", type=float, default=0.25)
    parser.add_argument("--start-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true", help="Without this, only validate")
    parser.add_argument("--output")
    args = parser.parse_args()
    if not 0.02 <= args.max_joint_speed <= 0.5:
        raise ValueError("max joint speed outside demo range")
    plan = json.loads(Path(args.plan).read_text())
    if not plan.get("plan_success"):
        raise ValueError("Plan did not succeed")
    names = plan["start_joint_names"]
    points = plan["trajectory_joint_positions_rad"]

    rclpy.init()
    node = rclpy.create_node("ur7e_hover_execute")
    latest: dict[str, JointState] = {}
    node.create_subscription(
        JointState, "/joint_states", lambda m: latest.__setitem__("js", m), qos_profile_sensor_data
    )

    def live() -> tuple[list[float], list[float]]:
        latest.pop("js", None)
        deadline = time.monotonic() + 5.0
        while "js" not in latest and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if "js" not in latest:
            raise RuntimeError("No /joint_states")
        m = latest["js"]
        index = [list(m.name).index(n) for n in names]
        return [m.position[i] for i in index], [m.velocity[i] for i in index]

    q, v = live()
    start_error = max(abs(a - b) for a, b in zip(q, points[0], strict=True))
    report: dict[str, Any] = {
        "start_error_deg": math.degrees(start_error),
        "max_live_speed": max(abs(x) for x in v),
    }
    if start_error > math.radians(args.start_tolerance_deg):
        raise RuntimeError(f"Live state differs from plan start: {report}")
    if report["max_live_speed"] > 0.005:
        raise RuntimeError(f"Robot not stationary: {report}")

    # Re-time: each segment takes long enough that no joint exceeds the cap, with
    # a 1.5x margin for the controller's interpolation overshoot.
    times = [0.0]
    for a, b in pairwise(points):
        step = max(abs(x - y) for x, y in zip(a, b, strict=True))
        times.append(times[-1] + max(1.5 * step / args.max_joint_speed, 0.02))
    # Hold the first and last segments longer so motion starts/ends gently.
    ramp = 2.0
    times = [t + (ramp if i else 0.0) for i, t in enumerate(times)]
    times[-1] += ramp
    report.update({"points": len(points), "duration_s": times[-1]})
    print(json.dumps({"stage": "VALIDATED", **report}), flush=True)
    if not args.execute:
        return

    client = ActionClient(
        node, FollowJointTrajectory, f"/{args.controller}/follow_joint_trajectory"
    )
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("Trajectory action server unavailable")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    goal.trajectory.points.append(JointTrajectoryPoint(positions=q, velocities=[0.0] * 6))
    for t, p in zip(times[1:], points[1:], strict=True):
        point = JointTrajectoryPoint(positions=p)
        _duration(point.time_from_start, t)
        goal.trajectory.points.append(point)
    goal.trajectory.points[-1].velocities = [0.0] * 6
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("Goal rejected")
    print(json.dumps({"stage": "EXECUTING", "duration_s": times[-1]}), flush=True)
    started = time.monotonic()
    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=times[-1] + 30.0)
    except KeyboardInterrupt:
        handle.cancel_goal_async()
        rclpy.spin_once(node, timeout_sec=0.5)
        raise
    if not result_future.done():
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        raise RuntimeError("Timed out; goal cancelled")
    result = result_future.result()
    final, final_v = live()
    report.update(
        {
            "stage": "FINISHED",
            "action_status": result.status,
            "error_code": result.result.error_code,
            "elapsed_s": time.monotonic() - started,
            "final_joint_positions_rad": final,
            "final_error_deg": math.degrees(
                max(abs(a - b) for a, b in zip(final, points[-1], strict=True))
            ),
            "final_max_speed": max(abs(x) for x in final_v),
        }
    )
    print(json.dumps(report), flush=True)
    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
