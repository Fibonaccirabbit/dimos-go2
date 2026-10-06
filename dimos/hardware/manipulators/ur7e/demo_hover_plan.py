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

"""Offline cuRobo hover preview. No ROS, action client, SSH, or robot commands.

The static camera fit and partial scene are UNVALIDATED. Successful optimization
is not permission to execute, nor a collision/safety certification for the room.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

from curobo._src.types.robot import RobotCfg  # type: ignore[import-not-found]
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg  # type: ignore[import-not-found]
from curobo.types import DeviceCfg, GoalToolPose, JointState, Pose  # type: ignore[import-not-found]
import numpy as np
import torch
import yaml  # type: ignore[import-untyped]


def _jsonable(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", required=True)
    parser.add_argument("--calibration-candidates", required=True)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--target-color-optical", nargs=3, type=float, required=True)
    parser.add_argument("--clearance", type=float, default=0.1)
    parser.add_argument(
        "--start-joints", nargs=6, type=float, help="Live start state; defaults to the sample"
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    started = time.monotonic()
    sample = np.load(args.sample)
    candidates = json.loads(Path(args.calibration_candidates).read_text())
    base_from_camera = np.asarray(
        candidates["candidates"][0]["base_link_from_color_optical"], dtype=float
    )
    point = np.asarray(args.target_color_optical, dtype=float)
    if base_from_camera.shape != (4, 4) or not np.isfinite(base_from_camera).all():
        raise ValueError("Invalid camera candidate")
    if not np.isfinite(point).all() or not 0.05 <= args.clearance <= 0.3:
        raise ValueError("Invalid target or preview clearance")
    surface = (base_from_camera @ np.r_[point, 1.0])[:3]
    hover = surface + np.array([0.0, 0.0, args.clearance])
    # Refuse targets off the workbench or outside comfortable reach before planning.
    if not (-0.05 <= surface[2] <= 0.4 and 0.25 <= float(np.hypot(*surface[:2])) <= 0.85):
        raise ValueError(f"Target outside the workbench reach envelope: {surface.tolist()}")
    config = yaml.safe_load(Path(args.robot_config).read_text())
    # The workstation YAML is a serialized loader config; RobotCfg.create passes
    # these runtime options explicitly, so retaining them causes duplicate kwargs.
    config["kinematics"].pop("load_collision_spheres", None)
    config["kinematics"].pop("num_envs", None)
    config["kinematics"]["tool_frames"] = ["gripper_tcp"]
    config["kinematics"]["collision_sphere_buffer"] = 0.01
    # Measured table surface is approximately z=0 in this UNVALIDATED static fit.
    # These are approximate boxes, not a full scene or a live obstacle monitor.
    scene = {
        "cuboid": {
            "preview_table": {
                "dims": [0.8, 0.9, 0.1],
                "pose": [-0.65, -0.05, -0.05, 1.0, 0.0, 0.0, 0.0],
            },
            "preview_target": {
                "dims": [0.08, 0.06, max(float(surface[2]), 0.02)],
                "pose": [
                    float(surface[0]),
                    float(surface[1]),
                    max(float(surface[2]), 0.02) / 2.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            },
        }
    }
    device = DeviceCfg(device="cuda:0")
    start = args.start_joints if args.start_joints else sample["joint_positions_rad"]
    q = torch.as_tensor(start, device=device.device, dtype=device.dtype)
    robot = RobotCfg.create(config, device_cfg=device)
    limits = robot.kinematics.get_joint_limits()
    if q.numel() != len(limits.joint_names) or not bool(torch.isfinite(q).all()):
        raise ValueError("Invalid captured start configuration")
    if bool(torch.any(q < limits.position[0])) or bool(torch.any(q > limits.position[1])):
        raise ValueError("Captured configuration outside original joint limits")
    # Narrow the offline search to the current winding, never expand URDF limits
    # or post-hoc wrap a trajectory (which would invalidate collision checking).
    window = math.pi - 1e-3
    limits.position[0] = torch.maximum(limits.position[0], q - window)
    limits.position[1] = torch.minimum(limits.position[1], q + window)
    if robot.kinematics.kinematics_config.cspace is not None:
        robot.kinematics.kinematics_config.cspace.default_joint_position = q.clone()
    print("Loading full arm/gripper collision model for OFFLINE preview", flush=True)
    planner = MotionPlanner(
        MotionPlannerCfg.create(
            robot,
            scene_model=scene,
            device_cfg=device,
            self_collision_check=True,
            num_ik_seeds=32,
            num_trajopt_seeds=4,
            position_tolerance=0.005,
            orientation_tolerance=0.05,
            optimizer_collision_activation_distance=0.02,
        )
    )
    if list(planner.joint_names) != list(limits.joint_names):
        raise ValueError("Planner joint order does not match captured start configuration")
    state = JointState.from_position(q.unsqueeze(0), joint_names=planner.joint_names)
    initial_tip = planner.compute_kinematics(state).tool_poses.get_link_pose("gripper_tcp")
    attempts: list[dict[str, Any]] = []
    output: dict[str, Any] = {
        "executed": False,
        "motion_allowed": False,
        "validation": "UNVALIDATED_CALIBRATION_AND_PARTIAL_SCENE",
        "base_frame": "base_link",
        "tool_frame": "gripper_tcp",
        "target_surface_base_link_m": surface.tolist(),
        "hover_gripper_tcp_base_link_m": hover.tolist(),
        "clearance_m": args.clearance,
        "start_joint_names": list(planner.joint_names),
        "start_joint_positions_rad": _jsonable(q),
        "initial_gripper_tcp_base_link_m": _jsonable(initial_tip.position),
        "gripper_locked_model_rad": config["kinematics"].get("lock_joints", {}),
        "offline_search_joint_limits_rad": _jsonable(limits.position),
        "scene": scene,
        "assumptions": [
            "Manual RGB surface patch is not a verified object center/top pose",
            "Camera candidate fitted to one static robot pose only",
            "No independent fiducial/reference calibration validation",
            "Table and target dimensions approximate; other objects/humans missing",
            "Gripper TCP/model closure and attachment offsets need physical validation",
            "Captured start state is not a fresh execution authorization",
            "Existing joint-smoke motion gate must not be bypassed with this preview",
        ],
        "attempts": attempts,
        "plan_success": False,
    }
    for yaw in (0.0, math.pi / 2.0):
        # Rz(yaw) Rx(pi): tool +Z points down; quaternion order is w,x,y,z.
        pose = Pose.from_list(
            [*hover.tolist(), 0.0, math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0],
            device_cfg=device,
        )
        goal = GoalToolPose.from_poses({"gripper_tcp": pose})
        print(f"Offline planning, downward tool yaw={math.degrees(yaw):.1f} deg", flush=True)
        result = planner.plan_pose(goal, state, max_attempts=3, enable_graph_attempt=1)
        success = result is not None and bool(torch.any(result.success))
        audit: dict[str, Any] = {
            "downward_tool_yaw_deg": math.degrees(yaw),
            "success": success,
            "diagnostics": _jsonable(planner.last_plan_diagnostics),
        }
        if result is not None:
            audit["position_error_m"] = _jsonable(result.position_error)
            audit["rotation_error_rad"] = _jsonable(result.rotation_error)
        attempts.append(audit)
        if success:
            plan = result.get_interpolated_plan()
            # Interpolated results include the locked gripper joint. Select the
            # active arm joints by name; never reshape seven joints into six.
            plan = planner.trajopt_solver.get_active_js(plan)
            positions = plan.position.reshape(-1, len(planner.joint_names))
            excursion = torch.amax(torch.abs(positions - q), dim=0)
            if (
                not bool(torch.isfinite(positions).all())
                or bool(torch.any(excursion > window + 1e-4))
                or bool(torch.any(positions < limits.position[0] - 1e-4))
                or bool(torch.any(positions > limits.position[1] + 1e-4))
            ):
                audit["preview_rejected"] = "Interpolated path violates offline winding window"
                continue
            trajectory_state = JointState.from_position(positions, joint_names=planner.joint_names)
            tip_path = planner.compute_kinematics(trajectory_state).tool_poses.get_link_pose(
                "gripper_tcp"
            )
            terminal_tip = tip_path.position.reshape(-1, 3)[-1]
            output.update(
                {
                    "plan_success": True,
                    "trajectory_joint_positions_rad": _jsonable(positions),
                    "trajectory_gripper_tcp_base_link_m": _jsonable(
                        tip_path.position.reshape(-1, 3)
                    ),
                    "nominal_trajectory_dt": _jsonable(plan.dt),
                    "terminal_gripper_tcp_base_link_m": _jsonable(terminal_tip),
                    "terminal_position_error_m": float(
                        torch.linalg.norm(terminal_tip - pose.position[0])
                    ),
                    "max_joint_excursion_deg": _jsonable(torch.rad2deg(excursion)),
                    "planned_final_joint_positions_rad": _jsonable(positions[-1]),
                }
            )
            break
    output["elapsed_s"] = time.monotonic() - started
    Path(args.output).write_text(json.dumps(output, indent=2))
    print(
        json.dumps(
            {key: value for key, value in output.items() if not key.startswith("trajectory_")}
        )
    )


if __name__ == "__main__":
    main()
