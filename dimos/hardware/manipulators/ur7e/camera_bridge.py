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

"""Read-only RGB-D ROS subscriptions over SSH, separate from the motion watchdog."""

import base64
from functools import partial
import json
from queue import Empty, Queue
import re
from threading import Thread
import time
from typing import Any
import uuid

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from dimos.hardware.manipulators.ur7e.policy import BridgeError
from dimos.hardware.manipulators.ur7e.ros_bridge import read_input
from dimos.hardware.manipulators.ur7e.target_geometry import transform_matrix


class CameraBridge:
    def __init__(self) -> None:
        self.node = Node("dimos_camera_bridge_" + uuid.uuid4().hex[:8])
        self._converter = CvBridge()
        self._images: dict[str, tuple[Any, float]] = {}
        self._info: dict[str, dict[str, Any]] = {}
        self._subscriptions: list[Any] = []
        self._counts = {"color": 0, "depth": 0}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

    def _image(self, kind: str, message: Image) -> None:
        self._images[kind] = (message, time.monotonic())
        self._counts[kind] += 1

    def _camera_info(self, kind: str, message: CameraInfo) -> None:
        self._info[kind] = {
            "width": message.width,
            "height": message.height,
            "distortion_model": message.distortion_model,
            "D": list(message.d),
            "K": list(message.k),
            "R": list(message.r),
            "P": list(message.p),
            "frame_id": message.header.frame_id,
        }

    def configure(self, namespace: str) -> dict[str, Any]:
        if not re.fullmatch(r"/[A-Za-z0-9_/]+", namespace):
            raise BridgeError("INVALID_INPUT", "Camera namespace must be an absolute ROS name")
        if self._subscriptions:
            raise BridgeError("ALREADY_CONFIGURED", "Camera subscriptions already configured")
        for kind in ("color", "depth"):
            self._subscriptions.extend(
                [
                    self.node.create_subscription(
                        Image,
                        f"{namespace}/{kind}/image_raw",
                        partial(self._image, kind),
                        qos_profile_sensor_data,
                    ),
                    self.node.create_subscription(
                        CameraInfo,
                        f"{namespace}/{kind}/camera_info",
                        partial(self._camera_info, kind),
                        qos_profile_sensor_data,
                    ),
                ]
            )
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and len(self._images) < 2:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.frame()

    def frame(self) -> dict[str, Any]:
        result: dict[str, Any] = {"frames_received": dict(self._counts), "registered": False}
        for kind in ("color", "depth"):
            if kind not in self._images:
                raise BridgeError("NO_CAMERA_FRAME", f"No {kind} image received")
            message, received = self._images[kind]
            age = time.monotonic() - received
            if age > 1.0:
                raise BridgeError("STALE_CAMERA_FRAME", f"{kind} image is stale: {age:.2f}s")
            if kind == "depth" and message.encoding != "16UC1":
                raise BridgeError("DEPTH_ENCODING", "Expected lossless 16UC1 depth")
            array = self._converter.imgmsg_to_cv2(
                message, "bgr8" if kind == "color" else "passthrough"
            )
            ok, encoded = cv2.imencode(
                ".jpg" if kind == "color" else ".png",
                array,
                [cv2.IMWRITE_JPEG_QUALITY, 80] if kind == "color" else [],
            )
            if not ok:
                raise BridgeError("IMAGE_ENCODING", "Could not encode camera image")
            result[kind] = {
                "data": base64.b64encode(encoded).decode("ascii"),
                "encoding": message.encoding,
                "width": message.width,
                "height": message.height,
                "frame_id": message.header.frame_id,
                "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                "age_s": age,
                "camera_info": self._info.get(kind),
            }
        result["stamp_skew_s"] = abs(result["color"]["stamp_s"] - result["depth"]["stamp_s"])
        result["color_from_depth"] = None
        try:
            transform = self._tf_buffer.lookup_transform(
                result["color"]["frame_id"], result["depth"]["frame_id"], rclpy.time.Time()
            ).transform
            t, q = transform.translation, transform.rotation
            result["color_from_depth"] = transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        except (TransformException, BridgeError) as exc:
            result["camera_extrinsics_error"] = str(exc)
        return result

    def dispatch(self, request: dict[str, Any]) -> dict[str, Any]:
        if request["op"] == "configure":
            return self.configure(request["namespace"])
        if request["op"] == "frame":
            return self.frame()
        raise BridgeError("UNSUPPORTED", "Camera bridge is read-only; no control operations")


def main() -> None:
    rclpy.init()
    bridge = CameraBridge()
    inbox: Queue[dict[str, Any] | None] = Queue()
    reader = Thread(target=read_input, args=(inbox,), daemon=True)
    reader.start()
    try:
        while rclpy.ok():
            rclpy.spin_once(bridge.node, timeout_sec=0.01)
            try:
                request = inbox.get_nowait()
            except Empty:
                continue
            if request is None:
                break
            response: dict[str, Any] = {"id": request["id"]}
            try:
                if "input_error" in request:
                    raise BridgeError("INVALID_INPUT", request["input_error"])
                response.update(ok=True, result=bridge.dispatch(request))
            except BridgeError as exc:
                response.update(ok=False, code=exc.code, message=str(exc))
            except (KeyError, TypeError, ValueError) as exc:
                response.update(ok=False, code="INVALID_INPUT", message=str(exc))
            print(json.dumps(response, allow_nan=False), flush=True)
    finally:
        bridge.node.destroy_node()
        rclpy.shutdown()
        reader.join(timeout=0.1)


if __name__ == "__main__":
    main()
