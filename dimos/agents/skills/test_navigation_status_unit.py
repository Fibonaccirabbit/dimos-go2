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

import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState


@pytest.fixture
def navigation(mocker):
    mocker.patch("dimos.models.vl.create.create")
    module = NavigationSkillContainer()
    module._navigation = mocker.MagicMock()
    module._skill_started = True
    yield module
    module.stop()


def test_idle_cancelled_goal_is_not_arrival(navigation):
    navigation._navigation.get_state.return_value = NavigationState.IDLE
    navigation._navigation.is_goal_reached.return_value = False
    assert navigation.navigation_status() == {
        "state": "idle",
        "goal_reached": False,
        "position": None,
    }


def test_arrival_status_includes_observed_odometry(navigation):
    navigation._navigation.get_state.return_value = NavigationState.IDLE
    navigation._navigation.is_goal_reached.return_value = True
    navigation._latest_odom = PoseStamped(position=Vector3(2.0, 3.0, 0.5))
    assert navigation.navigation_status() == {
        "state": "idle",
        "goal_reached": True,
        "position": {"x": 2.0, "y": 3.0, "z": 0.5},
    }


def test_rejected_goal_is_not_reported_as_started(navigation):
    navigation._navigation.set_goal.return_value = False
    assert "movement was not started" in navigation._navigate_to(PoseStamped(), "Found kitchen.")


def test_exact_embedding_match_is_not_discarded(navigation):
    pose = navigation._get_goal_pose_from_result(
        {"distance": 0.0, "metadata": [{"pos_x": 2.0, "pos_y": 3.0}]}
    )
    assert pose is not None
    assert (pose.x, pose.y) == (2.0, 3.0)
