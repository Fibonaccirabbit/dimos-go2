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

"""Small, bounded joint tests, not collision checking or servo control.

This module has no dimOS/ROS imports so the same policy runs on the Mac and
on the workstation without installing dimOS's heavy dependencies there.
"""

from dataclasses import dataclass
import math
from typing import Any, Literal

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
DOF = len(JOINT_NAMES)
NORMAL_SAFETY_MODE = 1
MIN_DURATION_S = 5.0
MAX_DURATION_S = 20.0
MAX_DELTA_RAD = math.radians(5.0)
MAX_VELOCITY_RAD_S = 0.01
MAX_FEEDBACK_AGE_S = 0.5
STATIONARY_VELOCITY_RAD_S = 0.005
POSITION_TOLERANCE_RAD = 0.001
ENVELOPE_MARGIN_RAD = 0.003
WATCHDOG_VELOCITY_RAD_S = 0.025
CLIENT_LEASE_S = 3.0
GOAL_GRACE_S = 10.0
DISCOVERY_TIMEOUT_S = 20.0
QUINTIC_PEAK_FACTOR = 1.875


def infer_effort_unit(
    parameters: dict[str, str], recipe_fields: set[str]
) -> Literal["A", "Nm", "unknown"]:
    """Old UR drivers expose motor current in JointState.effort, not Nm."""
    choice = parameters.get("use_currents_as_efforts", "").lower()
    if choice == "true" and "actual_current" in recipe_fields:
        return "A"
    if choice == "false" and "actual_current_as_torque" in recipe_fields:
        return "Nm"
    if choice:
        return "unknown"
    if "actual_current" in recipe_fields and "actual_current_as_torque" not in recipe_fields:
        return "A"
    if "actual_current_as_torque" in recipe_fields and "actual_current" not in recipe_fields:
        return "Nm"
    return "unknown"


class BridgeError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class JointSnapshot:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    age_s: float
    program_running: bool
    safety_mode: int
    position_lower: tuple[float, ...]
    position_upper: tuple[float, ...]
    velocity_max: tuple[float, ...]

    @classmethod
    def from_wire(cls, data: dict[str, Any], extra_age_s: float = 0.0) -> "JointSnapshot":
        if tuple(data["joint_names"]) != JOINT_NAMES:
            raise BridgeError("JOINT_ORDER", "Bridge joint order does not match UR7e")
        snapshot = cls(
            positions=tuple(data["positions"]),
            velocities=tuple(data["velocities"]),
            efforts=tuple(data["efforts"]),
            age_s=float(data["age_s"]) + extra_age_s,
            program_running=data["program_running"] is True,
            safety_mode=int(data["safety_mode"]),
            position_lower=tuple(data["position_lower"]),
            position_upper=tuple(data["position_upper"]),
            velocity_max=tuple(data["velocity_max"]),
        )
        snapshot.validate_feedback()
        return snapshot

    def validate_feedback(self) -> None:
        arrays = (
            self.positions,
            self.velocities,
            self.efforts,
            self.position_lower,
            self.position_upper,
            self.velocity_max,
        )
        if any(len(values) != DOF for values in arrays):
            raise BridgeError("INVALID_FEEDBACK", "Expected six entries per joint array")
        if not all(math.isfinite(v) for values in arrays for v in values):
            raise BridgeError("INVALID_FEEDBACK", "Non-finite joint feedback or limits")
        if not math.isfinite(self.age_s) or not 0 <= self.age_s <= MAX_FEEDBACK_AGE_S:
            raise BridgeError("STALE_FEEDBACK", "Joint feedback is stale")

    def validate_motion_ready(self) -> None:
        self.validate_feedback()
        if not self.program_running or self.safety_mode != NORMAL_SAFETY_MODE:
            raise BridgeError(
                "INVALID_STATE", "External Control must be running with safety NORMAL"
            )
        if max(abs(v) for v in self.velocities) > STATIONARY_VELOCITY_RAD_S:
            raise BridgeError("NOT_STATIONARY", "Robot must be stationary before this test")


def validate_target(snapshot: JointSnapshot, target: list[float], duration_s: float) -> None:
    snapshot.validate_motion_ready()
    if len(target) != DOF or not all(math.isfinite(v) for v in target):
        raise BridgeError("INVALID_INPUT", "Target must contain six finite angles in radians")
    if not math.isfinite(duration_s) or not MIN_DURATION_S <= duration_s <= MAX_DURATION_S:
        raise BridgeError(
            "INVALID_INPUT", f"Duration must be {MIN_DURATION_S}..{MAX_DURATION_S} seconds"
        )
    for i, value in enumerate(target):
        if not snapshot.position_lower[i] <= value <= snapshot.position_upper[i]:
            raise BridgeError(
                "JOINT_LIMIT", f"{JOINT_NAMES[i]} target is outside calibrated URDF limits"
            )
        delta = abs(value - snapshot.positions[i])
        if delta > MAX_DELTA_RAD + 1e-12:
            raise BridgeError(
                "DELTA_LIMIT", "Adapter permits at most five degrees per joint per request"
            )
        limit = min(MAX_VELOCITY_RAD_S, snapshot.velocity_max[i])
        if limit <= 0 or QUINTIC_PEAK_FACTOR * delta / duration_s > limit:
            raise BridgeError(
                "VELOCITY_LIMIT", "Quintic trajectory exceeds the test velocity limit"
            )


def delta_target(
    snapshot: JointSnapshot, joint: int, degrees: float, duration_s: float
) -> list[float]:
    if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= DOF:
        raise BridgeError("INVALID_INPUT", "Joint must be an integer from one to six")
    if not math.isfinite(degrees):
        raise BridgeError("INVALID_INPUT", "Joint displacement must be finite")
    target = list(snapshot.positions)
    target[joint - 1] += math.radians(degrees)
    validate_target(snapshot, target, duration_s)
    return target


def watchdog_fault(
    snapshot: JointSnapshot,
    start: list[float],
    target: list[float],
    client_age_s: float,
    remaining_s: float,
) -> str | None:
    """Return a cancellation reason; movement readiness differs from standstill."""
    if client_age_s > CLIENT_LEASE_S:
        return "CLIENT_LEASE_EXPIRED"
    if remaining_s < 0:
        return "TRAJECTORY_TIMEOUT"
    try:
        snapshot.validate_feedback()
    except BridgeError as exc:
        return exc.code
    if not snapshot.program_running or snapshot.safety_mode != NORMAL_SAFETY_MODE:
        return "SAFETY_OR_PROGRAM_CHANGED"
    for actual, lower, upper in zip(snapshot.positions, start, target, strict=True):
        if (
            not min(lower, upper) - ENVELOPE_MARGIN_RAD
            <= actual
            <= max(lower, upper) + ENVELOPE_MARGIN_RAD
        ):
            return "MOVEMENT_ENVELOPE"
    if max(abs(v) for v in snapshot.velocities) > WATCHDOG_VELOCITY_RAD_S:
        return "VELOCITY_ENVELOPE"
    return None
