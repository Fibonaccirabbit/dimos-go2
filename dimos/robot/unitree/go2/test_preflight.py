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

import shlex
import subprocess

import pytest

from dimos.robot.unitree.go2 import demo_preflight, ros2_lidar
from dimos.robot.unitree.go2.ros2_lidar import (
    Ros2LidarConfig,
    check_remote_environment,
    ssh_command,
)


@pytest.fixture
def config(tmp_path):
    return Ros2LidarConfig(
        host="127.0.0.1",
        control_path=str(tmp_path / "ssh.sock"),
        ros_setup="/example/ROS environment/setup.bash",
    )


def test_preflight_releases_socket_and_does_not_claim_sensor_or_motion_acceptance(mocker, config):
    connection = mocker.patch.object(demo_preflight.socket, "create_connection")
    remote = mocker.patch.object(demo_preflight, "check_remote_environment")
    result = demo_preflight.check_connection("127.0.0.1", config)
    assert result["ready_to_start"] is True
    assert result["motion_enabled"] is False
    assert "remain unverified" in result["note"]
    connection.assert_called_once_with(("127.0.0.1", 9991), timeout=3)
    connection.return_value.__exit__.assert_called_once()
    remote.assert_called_once_with(config)


def test_closed_webrtc_is_not_reported_as_ready(mocker, config):
    mocker.patch.object(
        demo_preflight.socket, "create_connection", side_effect=TimeoutError("offline")
    )
    mocker.patch.object(demo_preflight, "check_remote_environment")
    result = demo_preflight.check_connection("127.0.0.1", config)
    assert result["ready_to_start"] is False
    assert result["checks"]["webrtc_tcp"] == {"ok": False, "error": "offline"}


def test_missing_ssh_authentication_blocks_readiness(mocker, config):
    mocker.patch.object(demo_preflight.socket, "create_connection")
    mocker.patch.object(
        demo_preflight, "check_remote_environment", side_effect=RuntimeError("login required")
    )
    result = demo_preflight.check_connection("127.0.0.1", config)
    assert result["ready_to_start"] is False
    assert result["checks"]["ssh_ros_environment"] == {"ok": False, "error": "login required"}


def test_remote_check_requires_known_host_and_noninteractive_authentication(mocker, config):
    command = mocker.patch.object(ros2_lidar.subprocess, "run")
    command.return_value.returncode = 0
    check_remote_environment(config)
    argv = command.call_args.args[0]
    assert "BatchMode=yes" in argv
    assert "StrictHostKeyChecking=yes" in argv
    script = shlex.split(argv[-1])[-1]
    assert script.startswith("set -e; source " + shlex.quote(config.ros_setup) + ";")
    assert "import numpy, rclpy" in script
    assert command.call_args.kwargs["stdin"] == subprocess.DEVNULL
    assert command.call_args.kwargs["timeout"] == 10


def test_remote_login_failure_surfaces(mocker, config):
    command = mocker.patch.object(ros2_lidar.subprocess, "run")
    command.return_value.returncode = 255
    command.return_value.stderr = "Permission denied (publickey,password)."
    with pytest.raises(RuntimeError, match="Reconnect SSH"):
        check_remote_environment(config)


@pytest.mark.parametrize(
    "error", [FileNotFoundError("ssh missing"), subprocess.TimeoutExpired("ssh", 10)]
)
def test_remote_check_is_bounded_and_reports_process_errors(mocker, config, error):
    mocker.patch.object(ros2_lidar.subprocess, "run", side_effect=error)
    with pytest.raises(RuntimeError, match="could not complete"):
        check_remote_environment(config)


def test_ssh_command_delimits_host_from_options(config):
    argv = ssh_command(config, "true")
    assert argv[-3:] == ["--", "unitree@127.0.0.1", "true"]
