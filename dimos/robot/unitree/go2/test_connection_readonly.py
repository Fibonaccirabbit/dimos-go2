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

from unittest.mock import MagicMock

import pytest
from reactivex import Subject

from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.go2 import connection as conn_mod
from dimos.robot.unitree.go2.connection import GO2Connection


@pytest.fixture
def sensor_module(mocker):
    driver = MagicMock()
    driver.odom_stream.return_value = Subject()
    driver.lowstate_stream.return_value = Subject()
    factory = mocker.patch.object(conn_mod, "make_connection", return_value=driver)
    module = GO2Connection(
        g=GlobalConfig(robot_ip="127.0.0.1"), read_only=True, camera=False, lidar=False
    )
    subscribe = mocker.patch.object(module.cmd_vel, "subscribe")
    yield module, driver, factory, subscribe
    module.stop()


def test_readonly_module_forwards_guard_and_keeps_startup_inert(sensor_module):
    module, driver, factory, subscribe = sensor_module
    module.start()
    assert factory.call_args.kwargs["read_only"] is True
    subscribe.assert_not_called()
    driver.standup.assert_not_called()
    driver.balance_stand.assert_not_called()
    driver.set_obstacle_avoidance.assert_not_called()
    driver.odom_stream.assert_called_once()
    driver.lowstate_stream.assert_called_once()


def test_readonly_module_stop_does_not_lie_down(sensor_module):
    module, driver, _, _ = sensor_module
    module.stop()
    driver.liedown.assert_not_called()
    driver.stop.assert_called_once()
