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

"""JSON-line RPC over SSH, using keys or an already authenticated master.

There is no public command port, credential storage, or dependency on ROS
on the Mac. Requests are short; a running movement is polled, not a blocking
RPC, so another skill can cancel it.
"""

import json
from queue import Empty, Queue
import re
import shlex
import subprocess
from threading import RLock, Thread
from typing import Any, Protocol
import uuid

from dimos.hardware.manipulators.ur7e.policy import DISCOVERY_TIMEOUT_S, BridgeError


class BridgeTransport(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def is_connected(self) -> bool: ...
    def request(self, operation: str, **params: Any) -> dict[str, Any]: ...


class SSHROSTransport:
    def __init__(
        self,
        address: str,
        remote_root: str,
        control_path: str | None = None,
        allow_motion: bool = False,
        controller: str = "scaled_joint_trajectory_controller",
        ros_setup: str = "/opt/ros/humble/setup.bash",
        timeout_s: float = DISCOVERY_TIMEOUT_S + 5.0,
        bridge_module: str = "dimos.hardware.manipulators.ur7e.ros_bridge",
    ) -> None:
        if not address or address.startswith("-") or any(c.isspace() for c in address):
            raise ValueError("address must be an SSH host or user@host")
        if not remote_root.startswith("/") or not ros_setup.startswith("/"):
            raise ValueError("remote_root and ros_setup must be absolute remote paths")
        if not re.fullmatch(r"[A-Za-z0-9_/]+", controller):
            raise ValueError("Invalid ROS controller name")
        if bridge_module not in {
            "dimos.hardware.manipulators.ur7e.ros_bridge",
            "dimos.hardware.manipulators.ur7e.camera_bridge",
        }:
            raise ValueError("Unsupported workstation bridge module")
        remote = (
            f"source {shlex.quote(ros_setup)} && cd {shlex.quote(remote_root)} && "
            f"exec python3 -u -m {bridge_module}"
        )
        if bridge_module.endswith(".ros_bridge"):
            remote += f" --controller {shlex.quote(controller)}"
            if allow_motion:
                remote += " --allow-motion"
        elif allow_motion:
            raise ValueError("Camera connections cannot enable motion")
        self._command = [
            "ssh",
            "-T",
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
        ]
        if control_path:
            self._command += ["-S", control_path]
        self._command += [address, "bash -lc " + shlex.quote(remote)]
        self._process: subprocess.Popen[str] | None = None
        self._responses: Queue[dict[str, Any] | BridgeError] = Queue()
        self._reader: Thread | None = None
        self._lock = RLock()
        self._timeout_s = timeout_s

    def connect(self) -> None:
        with self._lock:
            if self.is_connected():
                return
            if self._process is not None:
                self.close()
            self._responses = Queue()
            self._process = subprocess.Popen(
                self._command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self._reader = Thread(target=self._read_responses, name="ur7e-ssh-reader", daemon=True)
            self._reader.start()

    def _read_responses(self) -> None:
        process = self._process
        assert process is not None and process.stdout is not None
        try:
            for line in process.stdout:
                try:
                    message = json.loads(line)
                    if not isinstance(message, dict):
                        raise TypeError("Bridge response is not an object")
                except (ValueError, TypeError) as exc:
                    self._responses.put(BridgeError("PROTOCOL", str(exc)))
                    return
                self._responses.put(message)
        finally:
            self._responses.put(BridgeError("DISCONNECTED", "SSH bridge closed its output"))

    def is_connected(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def request(self, operation: str, **params: Any) -> dict[str, Any]:
        with self._lock:
            if not self.is_connected():
                raise BridgeError("DISCONNECTED", "SSH bridge is not connected")
            process = self._process
            assert process is not None and process.stdin is not None
            request_id = uuid.uuid4().hex
            try:
                process.stdin.write(
                    json.dumps({"id": request_id, "op": operation, **params}, allow_nan=False)
                    + "\n"
                )
                process.stdin.flush()
                response = self._responses.get(timeout=self._timeout_s)
            except (OSError, Empty) as exc:
                self.close()
                raise BridgeError(
                    "CONNECTION_LOST", "Bridge request failed; never retry a movement automatically"
                ) from exc
            if isinstance(response, BridgeError):
                self.close()
                raise response
            if response.get("id") != request_id:
                self.close()
                raise BridgeError("PROTOCOL", "Bridge response/request mismatch")
            if not response.get("ok"):
                raise BridgeError(
                    response.get("code", "BRIDGE_ERROR"), response.get("message", "Bridge failed")
                )
            result = response.get("result")
            if not isinstance(result, dict):
                raise BridgeError("PROTOCOL", "Bridge result is not an object")
            return result

    def close(self) -> None:
        with self._lock:
            process = self._process
            if process is not None:
                if process.stdin is not None:
                    process.stdin.close()
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                if self._reader is not None:
                    self._reader.join(timeout=2)
                    if self._reader.is_alive():
                        raise BridgeError("CLEANUP", "SSH reader did not stop")
                if process.stdout is not None:
                    process.stdout.close()
            self._process = None
            self._reader = None
