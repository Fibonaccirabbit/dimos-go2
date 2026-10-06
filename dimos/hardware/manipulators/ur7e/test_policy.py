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

from dataclasses import replace
import math

import pytest

from dimos.hardware.manipulators.ur7e.policy import (
    JOINT_NAMES,
    BridgeError,
    JointSnapshot,
    delta_target,
    infer_effort_unit,
    validate_target,
    watchdog_fault,
)


@pytest.fixture
def snapshot() -> JointSnapshot:
    return JointSnapshot(
        (0.0, -2.0, 1.6, 4.7, 1.4, -1.27),
        (0.0,) * 6,
        (0.0,) * 6,
        0.002,
        True,
        1,
        (-2 * math.pi,) * 6,
        (2 * math.pi,) * 6,
        (math.pi,) * 6,
    )


@pytest.mark.parametrize(
    ("parameters", "fields", "unit"),
    [
        ({}, {"actual_current"}, "A"),
        (
            {"use_currents_as_efforts": "false"},
            {"actual_current", "actual_current_as_torque"},
            "Nm",
        ),
        ({"use_currents_as_efforts": "true"}, {"actual_current", "actual_current_as_torque"}, "A"),
        ({}, {"actual_current", "actual_current_as_torque"}, "unknown"),
        ({}, set(), "unknown"),
    ],
)
def test_effort_units_are_not_inferred_from_field_name(
    parameters: dict[str, str], fields: set[str], unit: str
) -> None:
    assert infer_effort_unit(parameters, fields) == unit


def test_delta_uses_measured_pose_and_preserves_other_joints(snapshot: JointSnapshot) -> None:
    target = delta_target(snapshot, 6, 0.5, 5.0)
    assert target[:5] == list(snapshot.positions[:5])
    assert target[5] - snapshot.positions[5] == pytest.approx(math.radians(0.5))


@pytest.mark.parametrize("degrees", [5.01, -5.01, math.inf, math.nan])
def test_rejects_unbounded_delta(snapshot: JointSnapshot, degrees: float) -> None:
    with pytest.raises(BridgeError):
        delta_target(snapshot, 6, degrees, 5.0)


@pytest.mark.parametrize("joint", [0, 7, True, 1.5])
def test_rejects_invalid_joint(snapshot: JointSnapshot, joint: int) -> None:
    with pytest.raises(BridgeError, match="integer"):
        delta_target(snapshot, joint, 0.5, 5.0)


def test_five_degree_request_keeps_other_joints_and_speed_bound(snapshot: JointSnapshot) -> None:
    target = delta_target(snapshot, 6, 5.0, 20.0)
    assert target[:5] == list(snapshot.positions[:5])
    assert target[5] - snapshot.positions[5] == pytest.approx(math.radians(5.0))
    with pytest.raises(BridgeError, match="velocity limit"):
        delta_target(snapshot, 6, 5.0, 5.0)


@pytest.mark.parametrize("duration", [4.9, 20.1, math.inf, math.nan])
def test_rejects_invalid_duration(snapshot: JointSnapshot, duration: float) -> None:
    with pytest.raises(BridgeError):
        delta_target(snapshot, 6, 0.5, duration)


@pytest.mark.parametrize(
    "changes",
    [
        {"age_s": 0.6},
        {"age_s": math.nan},
        {"safety_mode": 2},
        {"program_running": False},
        {"velocities": (0.1,) * 6},
        {"positions": (math.nan,) * 6},
        {"efforts": (0.0,) * 5},
    ],
)
def test_rejects_unready_robot(snapshot: JointSnapshot, changes: dict) -> None:
    with pytest.raises(BridgeError):
        delta_target(replace(snapshot, **changes), 6, 0.5, 5.0)


def test_uses_live_urdf_limits(snapshot: JointSnapshot) -> None:
    limited = replace(snapshot, position_upper=(0.001,) * 6)
    with pytest.raises(BridgeError, match="URDF limits"):
        delta_target(limited, 1, 0.5, 5.0)


def test_applies_quintic_peak_velocity_limit(snapshot: JointSnapshot) -> None:
    limited = replace(snapshot, velocity_max=(0.001,) * 6)
    with pytest.raises(BridgeError, match="velocity limit"):
        delta_target(limited, 6, 0.5, 5.0)


def test_rejects_wrong_target_length(snapshot: JointSnapshot) -> None:
    with pytest.raises(BridgeError, match="six"):
        validate_target(snapshot, [0.0] * 5, 5.0)


def test_wire_rejects_joint_reordering(snapshot: JointSnapshot) -> None:
    data = {"joint_names": list(reversed(JOINT_NAMES))}
    with pytest.raises(BridgeError, match="order"):
        JointSnapshot.from_wire(data)


@pytest.mark.parametrize(
    ("changes", "age", "remaining", "expected"),
    [
        ({}, 3.1, 5.0, "CLIENT_LEASE_EXPIRED"),
        ({}, 0.1, -0.1, "TRAJECTORY_TIMEOUT"),
        ({"age_s": 0.6}, 0.1, 5.0, "STALE_FEEDBACK"),
        ({"program_running": False}, 0.1, 5.0, "SAFETY_OR_PROGRAM_CHANGED"),
        ({"safety_mode": 3}, 0.1, 5.0, "SAFETY_OR_PROGRAM_CHANGED"),
        ({"velocities": (0.03,) * 6}, 0.1, 5.0, "VELOCITY_ENVELOPE"),
        ({"positions": (math.nan,) * 6}, 0.1, 5.0, "INVALID_FEEDBACK"),
    ],
)
def test_watchdog_cancels_on_fault(
    snapshot: JointSnapshot, changes: dict, age: float, remaining: float, expected: str
) -> None:
    target = delta_target(snapshot, 6, 0.5, 5.0)
    assert (
        watchdog_fault(
            replace(snapshot, **changes), list(snapshot.positions), target, age, remaining
        )
        == expected
    )


def test_watchdog_detects_non_target_joint_movement(snapshot: JointSnapshot) -> None:
    target = delta_target(snapshot, 6, 0.5, 5.0)
    moved = list(snapshot.positions)
    moved[0] += 0.01
    assert (
        watchdog_fault(
            replace(snapshot, positions=tuple(moved)), list(snapshot.positions), target, 0.1, 5.0
        )
        == "MOVEMENT_ENVELOPE"
    )


def test_watchdog_allows_planned_motion(snapshot: JointSnapshot) -> None:
    target = delta_target(snapshot, 6, 0.5, 5.0)
    assert (
        watchdog_fault(
            replace(snapshot, positions=tuple(target), velocities=(0.0,) * 5 + (0.006,)),
            list(snapshot.positions),
            target,
            0.1,
            5.0,
        )
        is None
    )
