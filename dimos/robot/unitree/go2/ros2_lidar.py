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
"""Read-only EDU ROS 2 lidar over SSH; all mapping remains on the Mac."""

import json
from pathlib import Path
import shlex
import subprocess
from threading import Event, Lock, Thread
import time
from typing import Any, BinaryIO

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
MAX_POINTS = 500_000


def read_cloud_packet(stream: BinaryIO, expected_frame: str, frame_id: str) -> PointCloud2:
    header_line = stream.readline(4097)
    if not header_line:
        raise EOFError("ROS 2 SSH pointcloud stream closed")
    if len(header_line) > 4096 or not header_line.endswith(b"\n"):
        raise ValueError("Invalid pointcloud packet header")
    header = json.loads(header_line)
    count = header["points"]
    if (
        header.get("version") != 1
        or header.get("frame") != expected_frame
        or not isinstance(count, int)
        or not 0 < count <= MAX_POINTS
        or header.get("bytes") != count * 12
    ):
        raise ValueError("Invalid pointcloud version, frame or payload size")
    payload = stream.read(count * 12)
    if len(payload) != count * 12:
        raise EOFError("Truncated pointcloud packet")
    points = np.frombuffer(payload, dtype="<f4").reshape(count, 3).copy()
    if not np.isfinite(points).all():
        raise ValueError("Non-finite pointcloud coordinates")
    # Go2 robot_pose and cloud_deskewed share ROS 'odom'. GO2Connection names
    # that unchanged coordinate system 'world'. This is an alias, not a TF.
    # Use receipt time, as the existing WebRTC odometry does; robot clock may differ.
    return PointCloud2.from_numpy(points, frame_id=frame_id, timestamp=time.time())


class Ros2LidarConfig(ModuleConfig):
    host: str = "192.168.123.18"
    user: str = "unitree"
    control_path: str = "/private/tmp/dimos-go2-edu-readonly.sock"
    ros_setup: str = "/home/unitree/cyclonedds_ws/install/setup.bash"
    topic: str = "/utlidar/cloud_deskewed"
    expected_frame: str = "odom"
    frame_id: str = "world"
    max_hz: float = Field(default=3.0, gt=0, le=15)


def ssh_command(cfg: Ros2LidarConfig, command: str) -> list[str]:
    """Use the operator's SSH authentication; never prompt or change host trust."""
    return [
        "ssh",
        "-T",
        "-S",
        cfg.control_path,
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=yes",
        "-o",
        "ConnectTimeout=5",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=2",
        "--",
        f"{cfg.user}@{cfg.host}",
        command,
    ]


def check_remote_environment(cfg: Ros2LidarConfig) -> None:
    """Verify SSH and existing ROS imports, without creating nodes or writing files."""
    script = (
        f"set -e; source {shlex.quote(cfg.ros_setup)}; "
        "python3 -c 'import numpy, rclpy; from sensor_msgs.msg import PointCloud2'"
    )
    try:
        result = subprocess.run(
            ssh_command(cfg, "bash -lc " + shlex.quote(script)),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError("Go2 SSH/ROS preflight could not complete; no reader started") from exc
    if result.returncode:
        raise RuntimeError(
            "Go2 SSH/ROS preflight failed. Reconnect SSH after battery replacement and "
            "verify ros_setup and the existing ROS dependencies. " + result.stderr.strip()[:500]
        )


class Go2Ros2Lidar(Module):
    config: Ros2LidarConfig
    lidar: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._process: subprocess.Popen[bytes] | None = None
        self._threads: list[Thread] = []
        self._stop = Event()
        self._lock = Lock()
        self._frames = 0
        self._points = 0
        self._last_received: float | None = None
        self._error: str | None = None

    @rpc
    def start(self) -> None:
        check_remote_environment(self.config)
        super().start()
        cfg = self.config
        reader = Path(__file__).with_name("ros2_lidar_reader.py").read_bytes()
        script = (
            f"set -e; source {shlex.quote(cfg.ros_setup)}; "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
            "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>"
            '<NetworkInterface name="eth0" /></Interfaces></General></Domain></CycloneDDS>\'; '
            f"exec python3 -u - {shlex.quote(cfg.topic)} {shlex.quote(cfg.expected_frame)} {cfg.max_hz}"
        )
        self._process = subprocess.Popen(
            ssh_command(cfg, "bash -lc " + shlex.quote(script)),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        assert self._process.stdin is not None
        self._process.stdin.write(reader)
        self._process.stdin.close()
        self._stop.clear()
        self._threads = [
            Thread(target=self._read, name="go2-ros2-lidar", daemon=True),
            Thread(target=self._read_stderr, name="go2-ros2-lidar-stderr", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def _read(self) -> None:
        assert self._process is not None and self._process.stdout is not None
        try:
            while not self._stop.is_set():
                cloud = read_cloud_packet(
                    self._process.stdout, self.config.expected_frame, self.config.frame_id
                )
                with self._lock:
                    self._frames += 1
                    self._points = len(cloud)
                    self._last_received = time.monotonic()
                self.lidar.publish(cloud)
        except (EOFError, ValueError, OSError, KeyError, TypeError) as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = str(exc)
                logger.error("ROS 2 lidar bridge stopped: %s", exc)

    def _read_stderr(self) -> None:
        assert self._process is not None and self._process.stderr is not None
        for line in self._process.stderr:
            logger.warning("Go2 ROS 2 reader: %s", line.decode(errors="replace").strip())

    @skill
    def pointcloud_status(self) -> dict[str, Any]:
        """Read real lidar bridge health. This cannot move or configure the robot."""
        with self._lock:
            return {
                "source": self.config.topic,
                "input_frame": self.config.expected_frame,
                "frame": self.config.frame_id,
                "frames": self._frames,
                "points": self._points,
                "age_seconds": (
                    None if self._last_received is None else time.monotonic() - self._last_received
                ),
                "ssh_alive": self._process is not None and self._process.poll() is None,
                "error": self._error,
                "motion_enabled": False,
            }

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._process is not None:
            if self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=3)
            for thread in self._threads:
                thread.join(timeout=3)
            for stream in (self._process.stdout, self._process.stderr):
                if stream is not None:
                    stream.close()
        super().stop()
