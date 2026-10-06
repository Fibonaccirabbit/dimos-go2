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

"""Workstation-only ROS bridge. Run with system Python after sourcing ROS.

No HTTP listener, no power/recovery commands, no gripper, and no automatic
movement. A goal belongs to this connection; input EOF, feedback loss, a
three-second client lease expiry, or a safety fault cancels that goal only.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
from queue import Empty, Queue
import sys
from threading import Thread
import time
from typing import Any
import uuid
import xml.etree.ElementTree as ET

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_dashboard_msgs.msg import SafetyMode

from dimos.hardware.manipulators.ur7e.policy import (
    DISCOVERY_TIMEOUT_S,
    DOF,
    ENVELOPE_MARGIN_RAD,
    GOAL_GRACE_S,
    JOINT_NAMES,
    MAX_FEEDBACK_AGE_S,
    POSITION_TOLERANCE_RAD,
    STATIONARY_VELOCITY_RAD_S,
    BridgeError,
    JointSnapshot,
    delta_target,
    infer_effort_unit,
    validate_target,
    watchdog_fault,
)

COMPETING_CONTROLLERS = {
    "joint_trajectory_controller",
    "forward_velocity_controller",
    "forward_position_controller",
    "forward_effort_controller",
    "passthrough_trajectory_controller",
    "freedrive_mode_controller",
    "force_mode_controller",
}


@dataclass
class Motion:
    motion_id: str
    start: list[float]
    target: list[float]
    sent_at: float
    deadline: float
    goal_future: Any
    handle: Any = None
    result_future: Any = None
    cancel_future: Any = None
    cancel_reason: str | None = None
    result: dict[str, Any] | None = None
    samples: int = 0
    max_velocity: float = 0.0


class ROSBridge:
    def __init__(self, controller: str, allow_motion: bool) -> None:
        self.node = Node("dimos_ur7e_bridge_" + uuid.uuid4().hex[:8])
        self._controller = controller.strip("/")
        self._allow_motion = allow_motion
        self._positions: list[float] | None = None
        self._velocities: list[float] = []
        self._efforts: list[float] = []
        self._effort_unit = "unknown"
        self._received = 0.0
        self._program: bool | None = None
        self._safety: int | None = None
        self._scaling: float | None = None
        self._tcp: dict[str, float] | None = None
        self._tcp_received = 0.0
        self._tcp_frame = ""
        self._wrench: list[float] | None = None
        self._wrench_received = 0.0
        self._wrench_frame = ""
        self._limits: dict[str, tuple[float, float, float]] = {}
        self._motion: Motion | None = None
        self._last_client = time.monotonic()
        self._controller_active = False
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions = [
            self.node.create_subscription(
                JointState, "/joint_states", self._on_joints, qos_profile_sensor_data
            ),
            self.node.create_subscription(
                Bool, "/io_and_status_controller/robot_program_running", self._on_program, latched
            ),
            self.node.create_subscription(
                SafetyMode, "/io_and_status_controller/safety_mode", self._on_safety, latched
            ),
            self.node.create_subscription(
                Float64,
                "/speed_scaling_state_broadcaster/speed_scaling",
                self._on_scaling,
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                PoseStamped, "/tcp_pose_broadcaster/pose", self._on_tcp, qos_profile_sensor_data
            ),
            self.node.create_subscription(
                WrenchStamped,
                "/force_torque_sensor_broadcaster/wrench",
                self._on_wrench,
                qos_profile_sensor_data,
            ),
        ]
        self._action = ActionClient(
            self.node, FollowJointTrajectory, f"/{self._controller}/follow_joint_trajectory"
        )
        self._params = self.node.create_client(
            GetParameters, "/robot_state_publisher/get_parameters"
        )
        self._controllers = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    def _on_joints(self, message: JointState) -> None:
        p = dict(zip(message.name, message.position, strict=True))
        v = dict(zip(message.name, message.velocity, strict=True))
        e = dict(zip(message.name, message.effort, strict=True))
        if not all(n in p and n in v and n in e for n in JOINT_NAMES):
            return
        self._positions = [p[n] for n in JOINT_NAMES]
        self._velocities = [v[n] for n in JOINT_NAMES]
        self._efforts = [e[n] for n in JOINT_NAMES]
        self._received = time.monotonic()
        if self._motion is not None and self._motion.result is None:
            self._motion.samples += 1
            self._motion.max_velocity = max(
                self._motion.max_velocity, max(abs(x) for x in self._velocities)
            )

    def _on_program(self, message: Bool) -> None:
        self._program = message.data

    def _on_safety(self, message: SafetyMode) -> None:
        self._safety = message.mode

    def _on_scaling(self, message: Float64) -> None:
        self._scaling = message.data

    def _on_tcp(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        if not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            self._tcp = None
            return
        self._tcp = {
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "roll": math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            "pitch": math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        }
        self._tcp_received = time.monotonic()
        self._tcp_frame = message.header.frame_id

    def _on_wrench(self, message: WrenchStamped) -> None:
        f, t = message.wrench.force, message.wrench.torque
        self._wrench = [f.x, f.y, f.z, t.x, t.y, t.z]
        self._wrench_received = time.monotonic()
        self._wrench_frame = message.header.frame_id

    def _await(self, future: Any, timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.tick()
        if not future.done():
            raise BridgeError("ROS_TIMEOUT", "ROS service timed out")
        return future.result()

    def initialize(self) -> None:
        deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
        while time.monotonic() < deadline and (
            self._positions is None or self._program is None or self._safety is None
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self._positions is None or self._program is None or self._safety is None:
            raise BridgeError("NO_FEEDBACK", "ROS joint/program/safety discovery did not finish")
        if not self._params.wait_for_service(timeout_sec=5):
            raise BridgeError("NO_URDF", "Live calibrated robot description unavailable")
        request = GetParameters.Request()
        request.names = ["robot_description"]
        response = self._await(self._params.call_async(request), 5)
        root = ET.fromstring(response.values[0].string_value)
        parameters = {p.attrib["name"]: p.text or "" for p in root.findall(".//hardware/param")}
        recipe = parameters.get("output_recipe_filename")
        if recipe is not None and Path(recipe).is_absolute():
            fields = set(Path(recipe).read_text().splitlines())
            self._effort_unit = infer_effort_unit(parameters, fields)
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            name = joint.get("name")
            if name in JOINT_NAMES and limit is not None:
                self._limits[name] = (
                    float(limit.attrib["lower"]),
                    float(limit.attrib["upper"]),
                    float(limit.attrib["velocity"]),
                )
        if len(self._limits) != DOF:
            raise BridgeError("NO_LIMITS", "Live URDF does not declare all six joint limits")
        self._check_controller(require_active=False)
        self.state()

    def _check_controller(self, require_active: bool = True) -> None:
        if not self._controllers.wait_for_service(timeout_sec=3):
            raise BridgeError("NO_CONTROLLER", "Controller manager unavailable")
        response = self._await(self._controllers.call_async(ListControllers.Request()), 3)
        active = {c.name for c in response.controller if c.state == "active"}
        self._controller_active = self._controller in active
        competitors = (active & COMPETING_CONTROLLERS) - {self._controller}
        if require_active and (not self._controller_active or competitors):
            raise BridgeError(
                "CONTROLLER_STATE",
                "Expected one active scaled trajectory controller, no competing motion controller",
            )

    def state(self) -> dict[str, Any]:
        now = time.monotonic()
        data = {
            "joint_names": list(JOINT_NAMES),
            "positions": self._positions,
            "velocities": self._velocities,
            "efforts": self._efforts,
            "effort_unit": self._effort_unit,
            "age_s": now - self._received,
            "program_running": self._program,
            "safety_mode": self._safety,
            "speed_scaling_percent": self._scaling,
            "position_lower": [self._limits[n][0] for n in JOINT_NAMES],
            "position_upper": [self._limits[n][1] for n in JOINT_NAMES],
            "velocity_max": [self._limits[n][2] for n in JOINT_NAMES],
            "controller": self._controller,
            "controller_active": self._controller_active,
            "motion_enabled": self._allow_motion,
            "motion_active": self._motion is not None and self._motion.result is None,
            "tcp_pose": self._tcp if now - self._tcp_received <= MAX_FEEDBACK_AGE_S else None,
            "tcp_frame_id": self._tcp_frame,
            "force_torque": self._wrench
            if now - self._wrench_received <= MAX_FEEDBACK_AGE_S
            else None,
            "force_torque_frame_id": self._wrench_frame,
        }
        JointSnapshot.from_wire(data)
        return data

    def _begin(self, target: list[float], duration_s: float) -> dict[str, Any]:
        if not self._allow_motion:
            raise BridgeError("READ_ONLY", "Bridge is read-only; no trajectory was sent")
        if self._motion is not None and self._motion.result is None:
            raise BridgeError("BUSY", "A trajectory is already running; it will not be preempted")
        self._check_controller()
        snapshot = JointSnapshot.from_wire(self.state())
        validate_target(snapshot, target, duration_s)
        if not self._action.wait_for_server(timeout_sec=3):
            raise BridgeError("NO_ACTION", "Trajectory action unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for positions, seconds in [(list(snapshot.positions), 0.0), (target, duration_s)]:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * DOF
            point.accelerations = [0.0] * DOF
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = round((seconds - int(seconds)) * 1_000_000_000)
            goal.trajectory.points.append(point)
        goal.path_tolerance = [
            JointTolerance(name=n, position=ENVELOPE_MARGIN_RAD) for n in JOINT_NAMES
        ]
        goal.goal_tolerance = [
            JointTolerance(name=n, position=POSITION_TOLERANCE_RAD) for n in JOINT_NAMES
        ]
        goal.goal_time_tolerance.sec = int(GOAL_GRACE_S)
        now = time.monotonic()
        motion_id = uuid.uuid4().hex
        self._motion = Motion(
            motion_id,
            list(snapshot.positions),
            target,
            now,
            now + duration_s + GOAL_GRACE_S,
            self._action.send_goal_async(goal),
        )
        return {"motion_id": motion_id, "pending": True, "accepted": False}

    def _cancel(self, reason: str) -> None:
        motion = self._motion
        if motion is None or motion.result is not None:
            return
        if motion.cancel_reason is None:
            motion.cancel_reason = reason
        if motion.handle is not None and motion.cancel_future is None:
            motion.cancel_future = motion.handle.cancel_goal_async()

    def tick(self) -> None:
        motion = self._motion
        if motion is None or motion.result is not None:
            return
        if motion.handle is None and motion.goal_future.done():
            motion.handle = motion.goal_future.result()
            if not motion.handle.accepted:
                motion.result = {
                    "done": True,
                    "success": False,
                    "code": "REJECTED",
                    "message": "Controller rejected trajectory",
                    "motion_id": motion.motion_id,
                }
                return
            motion.result_future = motion.handle.get_result_async()
        if motion.result_future is not None and motion.result_future.done():
            outcome = motion.result_future.result()
            final = list(self._positions or [])
            fresh = time.monotonic() - self._received <= MAX_FEEDBACK_AGE_S
            reached = len(final) == DOF and all(
                abs(a - b) <= POSITION_TOLERANCE_RAD
                for a, b in zip(final, motion.target, strict=True)
            )
            success = (
                outcome.status == 4
                and outcome.result.error_code == 0
                and fresh
                and reached
                and motion.cancel_reason is None
            )
            motion.result = {
                "done": True,
                "success": success,
                "motion_id": motion.motion_id,
                "action_status": outcome.status,
                "error_code": outcome.result.error_code,
                "message": outcome.result.error_string,
                "cancel_reason": motion.cancel_reason,
                "elapsed_s": time.monotonic() - motion.sent_at,
                "samples": motion.samples,
                "start_rad": motion.start,
                "target_rad": motion.target,
                "final_rad": final,
                "measured_delta_degrees": [
                    math.degrees(a - b) for a, b in zip(final, motion.start, strict=True)
                ],
                "max_velocity_rad_s": motion.max_velocity,
            }
            return
        now = time.monotonic()
        try:
            snapshot = JointSnapshot.from_wire(self.state())
            fault = watchdog_fault(
                snapshot,
                motion.start,
                motion.target,
                now - self._last_client,
                motion.deadline - now,
            )
        except BridgeError as exc:
            fault = exc.code
        if fault is not None:
            self._cancel(fault)
        if motion.cancel_reason is not None:
            self._cancel(motion.cancel_reason)

    def stop_owned_goal(self, reason: str = "OPERATOR_CANCEL") -> dict[str, Any]:
        self._cancel(reason)
        deadline = time.monotonic() + 3
        while (
            self._motion is not None and self._motion.result is None and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.tick()
        stopped = self._motion is None or self._motion.result is not None
        if stopped and self._positions is not None:
            stopped = (
                time.monotonic() - self._received <= MAX_FEEDBACK_AGE_S
                and max(abs(v) for v in self._velocities) <= STATIONARY_VELOCITY_RAD_S
            )
        return {"stopped": stopped, "scope": "this_bridge_owned_goal_only", "reason": reason}

    def dispatch(self, request: dict[str, Any]) -> dict[str, Any]:
        self._last_client = time.monotonic()
        operation = request["op"]
        if operation == "state":
            self._check_controller(require_active=False)
            return self.state()
        if operation in {"preview_delta", "move_delta"}:
            snapshot = JointSnapshot.from_wire(self.state())
            duration = float(request["duration_s"])
            target = delta_target(
                snapshot, request["joint"], float(request["delta_degrees"]), duration
            )
            if operation == "preview_delta":
                return {
                    "start_rad": list(snapshot.positions),
                    "target_rad": target,
                    "executed": False,
                    "collision_checked": False,
                }
            return self._begin(target, duration)
        if operation == "move":
            return self._begin(
                [float(v) for v in request["positions"]], float(request["duration_s"])
            )
        if operation == "result":
            if self._motion is None or self._motion.motion_id != request["motion_id"]:
                raise BridgeError("UNKNOWN_MOTION", "No such goal in this bridge session")
            if self._motion.result is not None:
                return self._motion.result
            return {
                "done": False,
                "motion_id": self._motion.motion_id,
                "accepted": self._motion.handle is not None and self._motion.handle.accepted,
                "cancel_reason": self._motion.cancel_reason,
            }
        if operation == "stop":
            return self.stop_owned_goal()
        raise BridgeError("UNSUPPORTED", "Unsupported bridge operation")


def reject_constant(value: str) -> None:
    raise ValueError("Non-finite JSON constant: " + value)


def read_input(inbox: Queue[dict[str, Any] | None]) -> None:
    for line in sys.stdin:
        try:
            if len(line) > 65536:
                raise ValueError("Request too large")
            request = json.loads(line, parse_constant=reject_constant)
            if not isinstance(request, dict) or not isinstance(request.get("id"), str):
                raise TypeError("Request must have an id")
        except (ValueError, TypeError) as exc:
            request = {"id": None, "input_error": str(exc)}
        inbox.put(request)
    inbox.put(None)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="scaled_joint_trajectory_controller")
    parser.add_argument("--allow-motion", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    bridge = ROSBridge(args.controller, args.allow_motion)
    inbox: Queue[dict[str, Any] | None] = Queue()
    reader = Thread(target=read_input, args=(inbox,), name="ur7e-stdin", daemon=True)
    try:
        bridge.initialize()
        reader.start()
        while rclpy.ok():
            rclpy.spin_once(bridge.node, timeout_sec=0.02)
            bridge.tick()
            try:
                request = inbox.get_nowait()
            except Empty:
                continue
            if request is None:
                break
            response: dict[str, Any] = {"id": request["id"]}
            try:
                if "input_error" in request:
                    raise BridgeError("INVALID_INPUT", request["input_error"])
                response.update(ok=True, result=bridge.dispatch(request))
            except BridgeError as exc:
                response.update(ok=False, code=exc.code, message=str(exc))
            except (KeyError, TypeError, ValueError) as exc:
                response.update(ok=False, code="INVALID_INPUT", message=str(exc))
            print(json.dumps(response, allow_nan=False), flush=True)
    finally:
        stopped = bridge.stop_owned_goal("CONNECTION_CLOSED")
        if not stopped["stopped"]:
            print("UR7e goal cancellation/standstill is unconfirmed", file=sys.stderr, flush=True)
        bridge.node.destroy_node()
        rclpy.shutdown()
        if reader.is_alive():
            reader.join(timeout=0.1)


if __name__ == "__main__":
    main()
