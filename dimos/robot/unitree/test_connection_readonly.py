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

from unittest.mock import AsyncMock, MagicMock

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree import connection as conn_mod
from dimos.robot.unitree.connection import UnitreeWebRTCConnection


@pytest.fixture
def sensor_connection(mocker):
    driver = MagicMock()
    driver.connect = AsyncMock()
    driver.disconnect = AsyncMock()
    driver.datachannel.disableTrafficSaving = AsyncMock()
    driver.datachannel.pub_sub.publish_request_new = AsyncMock()
    mocker.patch.object(conn_mod, "LegionConnection", return_value=driver)
    connection = UnitreeWebRTCConnection("127.0.0.1", read_only=True)
    yield connection, driver
    connection.stop()


def test_sensor_connection_connects_without_motion_switch(sensor_connection):
    connection, driver = sensor_connection
    driver.connect.assert_awaited_once()
    driver.datachannel.pub_sub.publish_request_new.assert_not_called()
    assert connection.thread.is_alive()


@pytest.mark.parametrize(
    ("method", "args"),
    [
        ("move", (Twist(),)),
        ("standup", ()),
        ("liedown", ()),
        ("balance_stand", ()),
        ("sport_command", (1002,)),
        ("set_motion_mode", ("ai",)),
        ("set_rage_mode", (True,)),
        ("set_obstacle_avoidance", (True,)),
        ("switch_joystick", (True,)),
        ("set_light", (5,)),
        ("publish_request", ("rt/api/sport/request", {"api_id": 1002})),
    ],
)
def test_sensor_connection_rejects_control_before_sending(sensor_connection, method, args):
    connection, driver = sensor_connection
    with pytest.raises(PermissionError, match="control is disabled"):
        getattr(connection, method)(*args)
    driver.datachannel.pub_sub.publish_request_new.assert_not_called()
    driver.datachannel.pub_sub.publish_without_callback.assert_not_called()
    assert connection.stop_timer is None


def test_sensor_connection_teardown_does_not_send_stop_or_motion(sensor_connection):
    connection, driver = sensor_connection
    connection.stop_movement()
    connection.stop()
    driver.disconnect.assert_awaited_once()
    driver.datachannel.pub_sub.publish_without_callback.assert_not_called()
    assert not connection.thread.is_alive()
