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

from collections.abc import Iterator
import math
from typing import Any

import pytest

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter
from dimos.hardware.manipulators.ur7e.adapter import UR7eROSAdapter
from dimos.hardware.manipulators.ur7e.policy import JOINT_NAMES, BridgeError
from dimos.hardware.manipulators.ur7e.transport import SSHROSTransport


@pytest.fixture
def state() -> dict[str, Any]:
    return {
        "joint_names": list(JOINT_NAMES),
        "positions": [0.0, -2.0, 1.6, 4.7, 1.4, -1.27],
        "velocities": [0.0] * 6,
        "efforts": [0.2] * 6,
        "effort_unit": "Nm",
        "age_s": 0.002,
        "program_running": True,
        "safety_mode": 1,
        "motion_active": False,
        "position_lower": [-2 * math.pi] * 6,
        "position_upper": [2 * math.pi] * 6,
        "velocity_max": [math.pi] * 6,
        "tcp_pose": {"x": 0.1, "y": 0.2, "z": 0.8, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "force_torque": [0.0] * 6,
    }


@pytest.fixture
def transport(mocker: Any, state: dict[str, Any]) -> Any:
    bridge = mocker.Mock()
    bridge.is_connected.return_value = True
    bridge.request.return_value = state
    return bridge


@pytest.fixture
def adapter(transport: Any) -> Iterator[UR7eROSAdapter]:
    arm = UR7eROSAdapter("workstation", "/bridge", transport=transport)
    yield arm
    arm.disconnect()


def test_implements_protocol_and_reads_actual_order(
    adapter: UR7eROSAdapter, transport: Any
) -> None:
    assert isinstance(adapter, ManipulatorAdapter)
    assert adapter.connect()
    assert adapter.read_joint_positions() == [0.0, -2.0, 1.6, 4.7, 1.4, -1.27]
    assert adapter.read_joint_efforts() == [0.2] * 6
    assert adapter.read_cartesian_position()["z"] == 0.8
    assert adapter.get_limits().position_upper[3] == 2 * math.pi
    transport.request.assert_called_once_with("state")


def test_connect_and_preview_never_move(adapter: UR7eROSAdapter, transport: Any) -> None:
    assert adapter.connect()
    preview = adapter.preview_joint_delta(6, 0.5)
    assert preview["executed"] is False
    assert preview["collision_checked"] is False
    assert preview["target_rad"][-1] == pytest.approx(-1.27 + math.radians(0.5))
    assert all(call.args == ("state",) for call in transport.request.call_args_list)


def test_read_only_is_enforced_before_transport(adapter: UR7eROSAdapter, transport: Any) -> None:
    assert not adapter.activate()
    transport.request.reset_mock()
    with pytest.raises(BridgeError, match="read-only"):
        adapter.move_joint_delta(6, 0.5)
    assert not adapter.write_joint_positions([0.0] * 6)
    transport.request.assert_not_called()


def test_rejects_servo_and_unsupported_physical_modes(
    adapter: UR7eROSAdapter, transport: Any
) -> None:
    assert adapter.set_control_mode(ControlMode.POSITION)
    assert not adapter.set_control_mode(ControlMode.SERVO_POSITION)
    assert not adapter.set_control_mode(ControlMode.TORQUE)
    assert not adapter.write_joint_velocities([0.0] * 6)
    assert not adapter.write_cartesian_position({"x": 1.0})
    assert not adapter.write_clear_errors()
    transport.request.assert_not_called()


def test_stale_feedback_is_not_replaced_by_fake_zeros(
    adapter: UR7eROSAdapter, transport: Any, state: dict[str, Any]
) -> None:
    state["age_s"] = 0.8
    with pytest.raises(BridgeError, match="stale"):
        adapter.read_joint_positions()


def test_cancel_reports_unconfirmed_stop(adapter: UR7eROSAdapter, transport: Any) -> None:
    transport.request.return_value = {"stopped": False}
    assert not adapter.write_stop()
    transport.request.assert_called_once_with("stop")


def test_motor_current_is_not_returned_as_torque(
    adapter: UR7eROSAdapter, state: dict[str, Any]
) -> None:
    state["effort_unit"] = "A"
    with pytest.raises(BridgeError, match="physical torque"):
        adapter.read_joint_efforts()
    assert adapter.snapshot()["efforts"] == [0.2] * 6


def test_operator_enabled_adapter_sends_relative_command(transport: Any) -> None:
    arm = UR7eROSAdapter("workstation", "/bridge", allow_motion=True, transport=transport)
    try:
        transport.request.return_value = {"motion_id": "test", "pending": True}
        assert arm.move_joint_delta(6, 0.5)["motion_id"] == "test"
        transport.request.assert_called_once_with(
            "move_delta", joint=6, delta_degrees=0.5, duration_s=5.0
        )
    finally:
        arm.disconnect()


@pytest.mark.parametrize("address", ["-oProxyCommand=bad", "host command", ""])
def test_ssh_rejects_option_injection(address: str) -> None:
    with pytest.raises(ValueError):
        SSHROSTransport(address, "/bridge")
