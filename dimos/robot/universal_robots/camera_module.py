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

"""RGB-D streams, an optional loopback preview, and opt-in vision target localization."""

import base64
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from threading import Event, RLock, Thread
import time
from typing import Any

import cv2
import numpy as np
from openai import OpenAI, OpenAIError

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.manipulators.ur7e.policy import BridgeError
from dimos.hardware.manipulators.ur7e.target_geometry import (
    locate_rgb_roi,
    roi_points,
    top_surface_point,
)
from dimos.hardware.manipulators.ur7e.transport import SSHROSTransport
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

PREVIEW_HTML = """<!doctype html><meta charset="utf-8"><title>UR7e · RGB-D</title>
<style>body{background:#121923;color:#e7edf4;font:16px system-ui;margin:24px}
.views{display:flex;gap:16px;flex-wrap:wrap}img{width:640px;max-width:95vw;border-radius:8px}
pre{white-space:pre-wrap}small{color:#99b2c8}</style>
<h2>UR7e · Orbbec RGB-D 实时相机</h2><small>本机预览 · 无运动接口 · 未发送给模型 · RGB/深度尚未空间对齐</small>
<div class="views"><div><h3>RGB</h3><img id="color"></div>
<div><h3>深度可视化 (原始 16-bit 数据另行保留)</h3><img id="depth"></div></div><pre id="status"></pre>
<div id="target-section" hidden><h3>目标定位快照 (仅相机坐标, 未执行移动)</h3><img id="target"></div>
<script>async function update(){try{const s=await(await fetch('/status')).json();
document.getElementById('status').textContent=JSON.stringify(s,null,2);for(const k of ['color','depth'])
document.getElementById(k).src='/'+k+'.jpg?t='+Date.now();
if(s.target_preview){document.getElementById('target-section').hidden=false;
document.getElementById('target').src='/target.jpg?t='+s.target_preview.stamp_s;}
}catch(e){document.getElementById('status').textContent=String(e);}
setTimeout(update,250);}update();</script>"""


def decode_frame(data: dict[str, Any]) -> tuple[Image, Image]:
    images: list[Image] = []
    for kind, fmt in (("color", ImageFormat.BGR), ("depth", ImageFormat.DEPTH16)):
        item = data[kind]
        raw = base64.b64decode(item["data"], validate=True)
        array = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if array is None or array.shape[:2] != (item["height"], item["width"]):
            raise BridgeError("INVALID_CAMERA_FRAME", f"Invalid {kind} image shape")
        if kind == "depth" and (array.dtype != np.uint16 or array.ndim != 2):
            raise BridgeError("DEPTH_ENCODING", "Depth must remain lossless uint16")
        if kind == "color" and (array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3):
            raise BridgeError("COLOR_ENCODING", "Color must be three-channel uint8")
        images.append(Image(array, fmt, item["frame_id"], float(item["stamp_s"])))
    return images[0], images[1]


class UR7eCameraConfig(ModuleConfig):
    address: str
    remote_root: str
    control_path: str | None = None
    ros_setup: str = "/opt/ros/humble/setup.bash"
    namespace: str = "/ur7e_camera"
    fps: float = 5.0
    viewer_port: int | None = None
    depth_unit_m: float = 0.001
    # Zero-shot target localization (only used when the operator allows image upload).
    vision_model: str = "deepseek-flash"
    vision_base_url: str = "https://api.deepseek.com"
    vision_api_key_env: str = "DEEPSEEK_API_KEY"
    vision_votes: int = 3
    vision_timeout_s: float = 20.0
    # Local JSON with candidates[0].base_link_from_color_optical (camera-to-arm fit).
    # When set, located objects use their top face in the arm base frame.
    camera_calibration: str | None = None


class UR7eCameraModule(Module):
    config: UR7eCameraConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    target_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._transport: SSHROSTransport | None = None
        self._shutdown = Event()
        self._lock = RLock()
        self._thread: Thread | None = None
        self._server_thread: Thread | None = None
        self._server: ThreadingHTTPServer | None = None
        self._latest: dict[str, bytes] = {}
        self._status: dict[str, Any] = {"connected": False}
        self._received_at = 0.0
        self._published = 0
        self._frame: dict[str, Any] | None = None
        self._target: dict[str, Any] | None = None
        self._target_preview: bytes = b""

    @rpc
    def start(self) -> None:
        if not 0 < self.config.fps <= 15:
            raise ValueError("Camera preview fps must be 0..15")
        super().start()
        self._shutdown.clear()
        self._transport = SSHROSTransport(
            self.config.address,
            self.config.remote_root,
            self.config.control_path,
            ros_setup=self.config.ros_setup,
            bridge_module="dimos.hardware.manipulators.ur7e.camera_bridge",
        )
        try:
            self._transport.connect()
            self._publish(self._transport.request("configure", namespace=self.config.namespace))
            if self.config.viewer_port is not None:
                self._start_preview()
            self._thread = Thread(target=self._run, name="ur7e-camera", daemon=True)
            self._thread.start()
        except Exception:
            self.stop()
            raise

    def _run(self) -> None:
        while not self._shutdown.wait(1 / self.config.fps):
            try:
                assert self._transport is not None
                self._publish(self._transport.request("frame"))
            except (BridgeError, ValueError) as exc:
                with self._lock:
                    self._status = {"connected": False, "error": str(exc)}
                    self._latest.clear()

    def _publish(self, data: dict[str, Any]) -> None:
        color, depth = decode_frame(data)
        self.color_image.publish(color)
        self.depth_image.publish(depth)
        for kind, stream in (("color", self.camera_info), ("depth", self.depth_camera_info)):
            info = data[kind].get("camera_info")
            if info is not None:
                stream.publish(CameraInfo(**info, ts=data[kind]["stamp_s"]))
        valid = depth.data[depth.data > 0]
        visualization = cv2.applyColorMap(
            np.clip(depth.data.astype(np.float32) / 3000 * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_TURBO,
        )
        visualization[depth.data == 0] = 0
        ok, preview = cv2.imencode(".jpg", visualization)
        if not ok:
            raise BridgeError("IMAGE_ENCODING", "Could not encode depth preview")
        with self._lock:
            self._frame = data
            self._published += 1
            self._received_at = time.monotonic()
            self._latest = {
                "/color.jpg": base64.b64decode(data["color"]["data"]),
                "/depth.jpg": preview.tobytes(),
                "/depth.png": base64.b64decode(data["depth"]["data"]),
            }
            self._status = {
                "connected": True,
                "frames_published": self._published,
                "resolution": [color.width, color.height],
                "preview_fps_limit": self.config.fps,
                "depth_encoding": "16UC1",
                "depth_unit_m": self.config.depth_unit_m,
                "depth_valid_fraction": float(valid.size / depth.data.size),
                "depth_median_m": float(np.median(valid) * self.config.depth_unit_m)
                if valid.size
                else None,
                "frames_received": data["frames_received"],
                "stamp_skew_s": data["stamp_skew_s"],
                "color_frame": color.frame_id,
                "depth_frame": depth.frame_id,
                "color_camera_info": data["color"].get("camera_info"),
                "depth_camera_info": data["depth"].get("camera_info"),
                "registered": False,
                "images_uploaded_to_model": False,
                "depth_to_color_extrinsics_available": data.get("color_from_depth") is not None,
                "target_preview": self._target,
            }

    def _current_status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
            result["age_s"] = time.monotonic() - self._received_at if self._received_at else None
            if result["age_s"] is not None and result["age_s"] > 2:
                result["connected"] = False
            return result

    @skill
    def ur7e_camera_status(self) -> SkillResult:
        """Read local RGB-D stream health and intrinsics only. Does not return pixels or upload images to a model."""
        data = self._current_status()
        if not data["connected"]:
            return SkillResult.fail("CAMERA_UNAVAILABLE", str(data))
        return SkillResult.ok("Local RGB-D streams are live; no image uploaded", camera=data)

    @skill
    def ur7e_preview_target_roi(self, left: int, top: int, right: int, bottom: int) -> SkillResult:
        """Locate an operator-selected RGB target patch in camera coordinates only. NO motion or base-coordinate guarantee.

        Args:
            left: RGB ROI left edge in pixels.
            top: RGB ROI top edge in pixels.
            right: RGB ROI right edge, exclusive.
            bottom: RGB ROI bottom edge, exclusive.
        """
        try:
            data = self._fresh_frame()
            result = self._locate_roi(
                data, [left, top, right, bottom], "TARGET PATCH - CAMERA ONLY / NO MOTION"
            )
            return SkillResult.ok(
                "Measured camera-space target only; base calibration and motion are NOT validated",
                target=result,
            )
        except (BridgeError, TypeError, ValueError) as exc:
            return SkillResult.fail(
                exc.code if isinstance(exc, BridgeError) else "INVALID_TARGET", str(exc)
            )

    def _fresh_frame(self, wait_s: float = 3.0) -> dict[str, Any]:
        # Single frames occasionally pair RGB and depth from different captures;
        # wait briefly for a synchronized one instead of failing on the first.
        deadline = time.monotonic() + wait_s
        while True:
            with self._lock:
                data = self._frame
                age = time.monotonic() - self._received_at
            if data is None or age > 1.0:
                error = BridgeError("STALE_CAMERA_FRAME", "A fresh RGB-D frame is required")
            elif data.get("color_from_depth") is None:
                error = BridgeError("NO_CAMERA_EXTRINSICS", "Depth-to-color TF is unavailable")
            elif data["stamp_skew_s"] > 0.01:
                error = BridgeError(
                    "UNSYNCED_RGBD", "RGB/depth timestamps differ by more than 10 ms"
                )
            else:
                return data
            if time.monotonic() >= deadline or self._shutdown.is_set():
                raise error
            self._shutdown.wait(0.05)

    def _base_from_camera(self) -> np.ndarray[Any, np.dtype[Any]] | None:
        if not self.config.camera_calibration:
            return None
        with open(self.config.camera_calibration) as f:
            matrix = np.asarray(
                json.load(f)["candidates"][0]["base_link_from_color_optical"], dtype=float
            )
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise BridgeError("INVALID_CALIBRATION", "Camera-to-arm calibration is malformed")
        return matrix

    def _locate_roi(
        self,
        data: dict[str, Any],
        roi: list[int],
        label: str,
        base_from_camera: np.ndarray[Any, np.dtype[Any]] | None = None,
    ) -> dict[str, Any]:
        color, depth = decode_frame(data)
        args = (
            depth.data,
            data["depth"]["camera_info"],
            data["color"]["camera_info"],
            data["color_from_depth"],
            roi,
            self.config.depth_unit_m,
        )
        if base_from_camera is None:
            result = locate_rgb_roi(*args)
        else:
            result = top_surface_point(roi_points(*args), base_from_camera)
            result["roi_rgb"] = roi
        result["frame_id"] = data["color"]["frame_id"]
        result["stamp_s"] = data["color"]["stamp_s"]
        overlay = color.data.copy()
        cv2.rectangle(overlay, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 255), 2)
        cv2.putText(overlay, label, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        ok, encoded = cv2.imencode(".jpg", overlay)
        if not ok:
            raise BridgeError("IMAGE_ENCODING", "Target preview encoding failed")
        with self._lock:
            self._target = result
            self._target_preview = encoded.tobytes()
        self.target_image.publish(Image(overlay, ImageFormat.BGR, color.frame_id, color.ts))
        return result

    def _vision_bbox(self, jpeg: bytes, description: str, width: int, height: int) -> list[int]:
        key = os.environ.get(self.config.vision_api_key_env, "")
        if not key:
            raise BridgeError("NO_VISION_MODEL", f"{self.config.vision_api_key_env} is not set")
        # Parallel votes, short timeout, no SDK retries: one slow request must not
        # stall the skill, and a lost vote is tolerated by the agreement check.
        client = OpenAI(
            api_key=key,
            base_url=self.config.vision_base_url,
            timeout=self.config.vision_timeout_s,
            max_retries=0,
        )
        # Normalized 0-1000 boxes with thinking disabled: about 1 s per answer and
        # consistent. Pixel-grid prompts made the model reason for up to 90 s and
        # still return y on a different scale.
        prompt = (
            f"Find: {description}. Return ONLY JSON "
            '{"found":true,"bbox":[x1,y1,x2,y2]} where coordinates are normalized to '
            "0-1000 (0,0 = top-left corner of the image, 1000,1000 = bottom-right), "
            'or {"found":false}.'
        )
        image = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()

        def ask() -> list[int] | None:
            reply = client.chat.completions.create(
                model=self.config.vision_model,
                temperature=0,
                extra_body={"thinking": {"type": "disabled"}},
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {"url": image}},
                        ],
                    }
                ],
            )
            text = reply.choices[0].message.content or ""
            try:
                parsed = json.loads(text[text.index("{") : text.rindex("}") + 1])
            except ValueError:
                return None
            box = parsed.get("bbox") if parsed.get("found") else None
            if not (isinstance(box, list) and len(box) == 4):
                return None
            x1, x2 = (round(float(v) * width / 1000) for v in (box[0], box[2]))
            y1, y2 = (round(float(v) * height / 1000) for v in (box[1], box[3]))
            x1, x2 = sorted((max(0, min(width, x1)), max(0, min(width, x2))))
            y1, y2 = sorted((max(0, min(height, y1)), max(0, min(height, y2))))
            return [x1, y1, x2, y2] if x2 - x1 >= 4 and y2 - y1 >= 4 else None

        votes = max(1, self.config.vision_votes)
        boxes: list[list[int]] = []
        errors: list[str] = []
        with ThreadPoolExecutor(max_workers=votes) as pool:
            for future in [pool.submit(ask) for _ in range(votes)]:
                try:
                    box = future.result()
                except OpenAIError as exc:
                    errors.append(type(exc).__name__)
                    continue
                if box is not None:
                    boxes.append(box)
        if not boxes and len(errors) == votes:
            raise BridgeError("VISION_MODEL_ERROR", f"All {votes} vision requests failed: {errors}")
        if not boxes:
            raise BridgeError("TARGET_NOT_FOUND", f"Vision model did not find: {description}")

        # Answers jitter by tens of pixels on small objects. Keep the answers whose
        # centres sit near the median centre and return their median box.
        centres = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for b in boxes])
        middle = np.median(centres, axis=0)
        sizes = np.array([max(b[2] - b[0], b[3] - b[1]) for b in boxes])
        near = np.linalg.norm(centres - middle, axis=1) <= max(30.0, float(np.median(sizes)))
        if near.sum() < min(2, len(boxes)):
            raise BridgeError("AMBIGUOUS_TARGET", f"Vision answers disagree: {boxes}")
        agreed = np.array(boxes)[near]
        return [int(v) for v in np.median(agreed, axis=0).round()]

    @skill
    def ur7e_locate_object(self, description: str) -> SkillResult:
        """Find an object in the workspace camera by description and measure its 3D top-surface point.

        Uploads the current RGB frame to the vision model. Returns the camera-optical
        point for ur7e_hover_above_point. Does not move the arm.

        Args:
            description: What to find, in English, e.g. "the white box on the metal table".
        """
        try:
            data = self._fresh_frame()
            item = data["color"]
            jpeg = base64.b64decode(item["data"], validate=True)
            box = self._vision_bbox(jpeg, description, item["width"], item["height"])
            base_from_camera = self._base_from_camera()
            cx, cy = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2
            if base_from_camera is None:
                # No arm calibration: measure only the box centre as a flat patch.
                hw, hh = max(2, (box[2] - box[0]) // 4), max(2, (box[3] - box[1]) // 4)
            else:
                # Whole box minus a margin; the top face is picked by height.
                hw, hh = max(2, (box[2] - box[0]) * 2 // 5), max(2, (box[3] - box[1]) * 2 // 5)
            roi = [cx - hw, cy - hh, cx + hw, cy + hh]
            result = self._locate_roi(data, roi, f"TARGET: {description}"[:60], base_from_camera)
            result["bbox_rgb"] = box
            return SkillResult.ok(
                "Target located from a fresh frame; pass point_color_optical_m to "
                "ur7e_hover_above_point",
                target=result,
            )
        except (BridgeError, TypeError, ValueError) as exc:
            return SkillResult.fail(
                exc.code if isinstance(exc, BridgeError) else "INVALID_TARGET", str(exc)
            )
        except OpenAIError as exc:
            return SkillResult.fail("VISION_MODEL_ERROR", f"{type(exc).__name__}: {exc}")

    def _start_preview(self) -> None:
        module = self

        class PreviewHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = self.path.split("?", 1)[0]
                content_type = "application/json"
                if path == "/":
                    body = PREVIEW_HTML.encode()
                    content_type = "text/html; charset=utf-8"
                elif path == "/status":
                    body = json.dumps(module._current_status()).encode()
                else:
                    with module._lock:
                        body = (
                            module._target_preview
                            if path == "/target.jpg"
                            else module._latest.get(path, b"")
                        )
                    if not body or not module._current_status()["connected"]:
                        self.send_error(503, "Camera frame unavailable")
                        return
                    content_type = "image/png" if path.endswith(".png") else "image/jpeg"
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format: str, *args: Any) -> None:
                pass

        assert self.config.viewer_port is not None
        self._server = ThreadingHTTPServer(("127.0.0.1", self.config.viewer_port), PreviewHandler)
        self._server_thread = Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

    @rpc
    def stop(self) -> None:
        self._shutdown.set()
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._server_thread is not None:
            self._server_thread.join(timeout=3)
            self._server_thread = None
        with self._lock:
            self._status = {"connected": False}
            self._latest.clear()
            self._frame = None
            self._target = None
            self._target_preview = b""
        super().stop()


ur7e_camera = UR7eCameraModule.blueprint
