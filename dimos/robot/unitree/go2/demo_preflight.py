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

"""Read-only startup checks, not a navigation or camera acceptance test."""

import argparse
import json
import socket
from typing import Any

from dimos.robot.unitree.go2.ros2_lidar import Ros2LidarConfig, check_remote_environment

WEBRTC_PORT = 9991


def check_connection(robot_ip: str, lidar_config: Ros2LidarConfig) -> dict[str, Any]:
    checks: dict[str, Any] = {}
    try:
        with socket.create_connection((robot_ip, WEBRTC_PORT), timeout=3):
            pass
    except OSError as exc:
        checks["webrtc_tcp"] = {"ok": False, "error": str(exc)}
    else:
        checks["webrtc_tcp"] = {"ok": True}
    try:
        check_remote_environment(lidar_config)
    except RuntimeError as exc:
        checks["ssh_ros_environment"] = {"ok": False, "error": str(exc)}
    else:
        checks["ssh_ros_environment"] = {"ok": True}
    return {
        "ready_to_start": all(check["ok"] for check in checks.values()),
        "motion_enabled": False,
        "checks": checks,
        "note": "Startup prerequisites only; live camera, pointcloud and safe path remain unverified.",
    }


def main() -> None:
    defaults = Ros2LidarConfig()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--lidar-host", default=defaults.host)
    parser.add_argument("--control-path", default=defaults.control_path)
    parser.add_argument("--ros-setup", default=defaults.ros_setup)
    args = parser.parse_args()
    result = check_connection(
        args.robot_ip,
        Ros2LidarConfig(
            host=args.lidar_host, control_path=args.control_path, ros_setup=args.ros_setup
        ),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["ready_to_start"] else 1)


if __name__ == "__main__":
    main()
