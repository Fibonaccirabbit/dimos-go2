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
import json
from typing import Any

import pytest

from dimos.hardware.manipulators.ur7e.policy import BridgeError
from dimos.robot.universal_robots.ur7e_skill_container import UR7eSkillContainer


@pytest.fixture
def mock_adapter(mocker: Any) -> Any:
    adapter = mocker.Mock()
    adapter.connect.return_value = True
    adapter.snapshot.return_value = {"positions": [1.0] * 6, "motion_enabled": False}
    mocker.patch(
        "dimos.robot.universal_robots.ur7e_skill_container.UR7eROSAdapter", return_value=adapter
    )
    return adapter


@pytest.fixture
def container(mock_adapter: Any) -> Iterator[UR7eSkillContainer]:
    module = UR7eSkillContainer(address="workstation", remote_root="/bridge")
    module.start()
    try:
        yield module
    finally:
        module.stop()


def test_skills_have_typed_mcp_schemas(container: UR7eSkillContainer) -> None:
    skills = {s.func_name: s for s in container.get_skills()}
    assert set(skills) == {
        "ur7e_status",
        "ur7e_preview_joint_delta",
        "ur7e_move_joint_delta",
        "ur7e_stop",
        "ur7e_hover_above_white_box",
        "ur7e_return_to_start",
        "ur7e_hover_above_point",
    }
    schema = json.loads(skills["ur7e_preview_joint_delta"].args_schema)
    assert schema["properties"]["joint"]["type"] == "integer"
    assert schema["properties"]["delta_degrees"]["type"] == "number"
    assert skills["ur7e_move_joint_delta"].uses == ("ur7e_motion",)
    assert skills["ur7e_stop"].uses == ()


def test_status_returns_measured_feedback(container: UR7eSkillContainer) -> None:
    outcome = container.ur7e_status()
    assert outcome.success
    assert outcome.metadata["state"]["positions"] == [1.0] * 6


def test_offline_skill_reports_not_configured() -> None:
    module = UR7eSkillContainer(address="workstation", remote_root="/bridge")
    try:
        result = module.ur7e_status()
        assert not result.success
        assert result.error_code == "NOT_CONFIGURED"
    finally:
        module.stop()


def test_move_reports_read_only_without_success_claim(
    container: UR7eSkillContainer, mock_adapter: Any
) -> None:
    mock_adapter.move_joint_delta.side_effect = BridgeError("READ_ONLY", "No trajectory sent")
    result = container.ur7e_move_joint_delta(6, 0.5)
    assert not result.success
    assert result.error_code == "READ_ONLY"


def test_plan_moves_refused_in_read_only_session(
    container: UR7eSkillContainer, mocker: Any
) -> None:
    run = mocker.patch("dimos.robot.universal_robots.ur7e_skill_container.subprocess.run")
    for outcome in (
        container.ur7e_hover_above_white_box(),
        container.ur7e_return_to_start(),
        container.ur7e_hover_above_point([0.3, 0.4, 1.4]),
    ):
        assert not outcome.success
        assert outcome.error_code == "READ_ONLY"
    run.assert_not_called()
