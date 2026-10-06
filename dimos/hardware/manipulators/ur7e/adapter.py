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

"""UR7e adapter for state and bounded, whole ROS trajectories.

POSITION commands are asynchronous complete trajectories. SERVO_POSITION is
intentionally rejected: a ROS action over SSH is not a 100 Hz servo channel.
Use the UR7e skills for this first-stage backend, not Drake/coordinator tasks.
"""

import math
from threading import RLock
import time
from typing import Any

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.manipulators.ur7e.policy import (
    DOF,
    MIN_DURATION_S,
    NORMAL_SAFETY_MODE,
    BridgeError,
    JointSnapshot,
    delta_target,
    validate_target,
)
from dimos.hardware.manipulators.ur7e.transport import BridgeTransport, SSHROSTransport
from dimos.hardware.spec import JointLimits
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
CACHE_TTL_S = 0.02


class UR7eROSAdapter:
    def __init__(
        self,
        address: str,
        remote_root: str,
        control_path: str | None = None,
        allow_motion: bool = False,
        dof: int = DOF,
        controller: str = "scaled_joint_trajectory_controller",
        ros_setup: str = "/opt/ros/humble/setup.bash",
        transport: BridgeTransport | None = None,
        **_unused: Any,
    ) -> None:
        if dof != DOF:
            raise ValueError("UR7e owns six arm joints; the gripper is not integrated")
        self._transport = transport or SSHROSTransport(
            address=address,
            remote_root=remote_root,
            control_path=control_path,
            allow_motion=allow_motion,
            controller=controller,
            ros_setup=ros_setup,
        )
        self._allow_motion = allow_motion
        self._cache: dict[str, Any] | None = None
        self._cached_at = 0.0
        self._lock = RLock()

    def connect(self) -> bool:
        try:
            self._transport.connect()
            self.snapshot(refresh=True)
        except (BridgeError, OSError) as exc:
            logger.error("UR7e bridge connection failed", error=str(exc))
            self._transport.close()
            return False
        return True

    def disconnect(self) -> None:
        # EOF makes the bridge cancel its own unfinished goal before exiting.
        self._transport.close()
        with self._lock:
            self._cache = None

    def is_connected(self) -> bool:
        return self._transport.is_connected()

    def snapshot(self, refresh: bool = False) -> dict[str, Any]:
        with self._lock:
            elapsed = time.monotonic() - self._cached_at
            if refresh or self._cache is None or elapsed >= CACHE_TTL_S:
                self._cache = self._transport.request("state")
                self._cached_at = time.monotonic()
                elapsed = 0.0
            data = dict(self._cache)
            data["age_s"] += elapsed
            JointSnapshot.from_wire(data)
            return data

    def activate(self) -> bool:
        # Readiness check only: no power, brake release, play or controller switch.
        try:
            JointSnapshot.from_wire(self.snapshot(refresh=True)).validate_motion_ready()
        except BridgeError:
            return False
        return self._allow_motion

    def deactivate(self) -> bool:
        return self.write_stop()

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(vendor="Universal Robots", model="UR7e", dof=DOF)

    def get_dof(self) -> int:
        return DOF

    def get_limits(self) -> JointLimits:
        state = JointSnapshot.from_wire(self.snapshot())
        return JointLimits(state.position_lower, state.position_upper, state.velocity_max)

    def set_control_mode(self, mode: ControlMode) -> bool:
        return mode == ControlMode.POSITION

    def get_control_mode(self) -> ControlMode:
        return ControlMode.POSITION

    def read_joint_positions(self) -> list[float]:
        return list(self.snapshot()["positions"])

    def read_joint_velocities(self) -> list[float]:
        return list(self.snapshot()["velocities"])

    def read_joint_efforts(self) -> list[float]:
        data = self.snapshot()
        if data.get("effort_unit") != "Nm":
            raise BridgeError(
                "UNSUPPORTED_EFFORT_UNIT",
                "Driver effort is motor current/unknown, not physical torque in Nm",
            )
        return list(data["efforts"])

    def read_state(self) -> dict[str, int]:
        state = self.snapshot()
        return {
            "mode": 0,
            "state": int(state["motion_active"]),
            "program_running": int(state["program_running"]),
            "safety_mode": state["safety_mode"],
            "motion_enabled": int(self._allow_motion),
        }

    def read_error(self) -> tuple[int, str]:
        try:
            state = self.snapshot()
        except BridgeError as exc:
            return 1, str(exc)
        if state["safety_mode"] != NORMAL_SAFETY_MODE:
            return state["safety_mode"], "Robot safety mode is not NORMAL"
        if not state["program_running"]:
            return 1, "External Control program is not running"
        return 0, ""

    def preview_joint_delta(
        self, joint: int, delta_degrees: float, duration_s: float = MIN_DURATION_S
    ) -> dict[str, Any]:
        state = JointSnapshot.from_wire(self.snapshot(refresh=True))
        target = delta_target(state, joint, delta_degrees, duration_s)
        return {
            "start_rad": list(state.positions),
            "target_rad": target,
            "joint": joint,
            "delta_degrees": delta_degrees,
            "duration_s": duration_s,
            "motion_enabled": self._allow_motion,
            "executed": False,
            "collision_checked": False,
        }

    def move_joint_delta(
        self, joint: int, delta_degrees: float, duration_s: float = MIN_DURATION_S
    ) -> dict[str, Any]:
        if not self._allow_motion:
            raise BridgeError("READ_ONLY", "This session is read-only; no trajectory was sent")
        # Relative moves are recalculated from fresh feedback on the workstation.
        return self._transport.request(
            "move_delta", joint=joint, delta_degrees=delta_degrees, duration_s=duration_s
        )

    def trajectory_result(self, motion_id: str) -> dict[str, Any]:
        return self._transport.request("result", motion_id=motion_id)

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if not self._allow_motion or not math.isfinite(velocity) or not 0 < velocity <= 1:
            return False
        duration = MIN_DURATION_S / velocity
        try:
            state = JointSnapshot.from_wire(self.snapshot(refresh=True))
            validate_target(state, positions, duration)
            self._transport.request("move", positions=positions, duration_s=duration)
        except BridgeError as exc:
            logger.warning("UR7e position command refused", code=exc.code, error=str(exc))
            return False
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_stop(self) -> bool:
        try:
            response = self._transport.request("stop")
        except BridgeError as exc:
            logger.error("UR7e cancellation failed", error=str(exc))
            return False
        return response["stopped"] is True

    def write_enable(self, enable: bool) -> bool:
        # Never power the robot or change operating mode from an agent.
        return self.activate() if enable else self.deactivate()

    def read_enabled(self) -> bool:
        state = self.snapshot()
        return (
            self._allow_motion
            and state["program_running"]
            and state["safety_mode"] == NORMAL_SAFETY_MODE
        )

    def write_clear_errors(self) -> bool:
        return False

    def read_cartesian_position(self) -> dict[str, float] | None:
        return self.snapshot().get("tcp_pose")

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return self.snapshot().get("force_torque")
