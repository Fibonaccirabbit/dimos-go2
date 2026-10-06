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

import base64
from typing import Any

import cv2
import numpy as np
import pytest

from dimos.hardware.manipulators.ur7e.policy import BridgeError
from dimos.hardware.manipulators.ur7e.transport import SSHROSTransport
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.robot.universal_robots.camera_module import UR7eCameraModule, decode_frame


@pytest.fixture
def frame() -> dict[str, Any]:
    color = np.full((2, 3, 3), 80, dtype=np.uint8)
    depth = np.array([[0, 800, 900], [1000, 3000, 65535]], dtype=np.uint16)
    result: dict[str, Any] = {"frames_received": {"color": 2, "depth": 2}, "stamp_skew_s": 0.0}
    for kind, array, extension in (("color", color, ".jpg"), ("depth", depth, ".png")):
        ok, encoded = cv2.imencode(extension, array)
        assert ok
        result[kind] = {
            "data": base64.b64encode(encoded).decode(),
            "width": 3,
            "height": 2,
            "frame_id": f"{kind}_optical_frame",
            "stamp_s": 123.5,
            "camera_info": None,
        }
    return result


def test_depth_is_lossless_with_source_time_and_frame(frame: dict[str, Any]) -> None:
    color, depth = decode_frame(frame)
    assert color.format == ImageFormat.BGR
    assert depth.format == ImageFormat.DEPTH16
    assert depth.data.dtype == np.uint16
    np.testing.assert_array_equal(depth.data, [[0, 800, 900], [1000, 3000, 65535]])
    assert depth.ts == 123.5
    assert depth.frame_id == "depth_optical_frame"


def test_rejects_mismatched_dimensions(frame: dict[str, Any]) -> None:
    frame["depth"]["width"] = 4
    with pytest.raises(BridgeError, match="shape"):
        decode_frame(frame)


def test_rejects_eight_bit_depth(frame: dict[str, Any]) -> None:
    ok, encoded = cv2.imencode(".png", np.zeros((2, 3), dtype=np.uint8))
    assert ok
    frame["depth"]["data"] = base64.b64encode(encoded).decode()
    with pytest.raises(BridgeError, match="uint16"):
        decode_frame(frame)


def test_camera_transport_cannot_enable_motion() -> None:
    with pytest.raises(ValueError, match="cannot enable motion"):
        SSHROSTransport(
            "workstation",
            "/bridge",
            allow_motion=True,
            bridge_module="dimos.hardware.manipulators.ur7e.camera_bridge",
        )


def test_status_never_returns_pixels_to_model(frame: dict[str, Any]) -> None:
    module = UR7eCameraModule(address="workstation", remote_root="/bridge")
    try:
        module._publish(frame)
        status = module.ur7e_camera_status()
        assert status.success
        assert status.metadata["camera"]["images_uploaded_to_model"] is False
        assert "data" not in str(status.metadata)
        assert status.metadata["camera"]["registered"] is False
        assert status.metadata["camera"]["frames_published"] == 1
    finally:
        module.stop()


def test_offline_camera_does_not_claim_stream_is_live() -> None:
    module = UR7eCameraModule(address="workstation", remote_root="/bridge")
    try:
        result = module.ur7e_camera_status()
        assert not result.success
        assert result.error_code == "CAMERA_UNAVAILABLE"
    finally:
        module.stop()


def _vision_module(mocker: Any, answers: list[Any]) -> UR7eCameraModule:
    mocker.patch.dict("os.environ", {"DEEPSEEK_API_KEY": "test"})
    replies = iter(answers)

    def create(**_: Any) -> Any:
        answer = next(replies)
        if isinstance(answer, Exception):
            raise answer
        return mocker.Mock(choices=[mocker.Mock(message=mocker.Mock(content=answer))])

    client = mocker.Mock()
    client.chat.completions.create.side_effect = create
    mocker.patch("dimos.robot.universal_robots.camera_module.OpenAI", return_value=client)
    return UR7eCameraModule(address="workstation", remote_root="/bridge", vision_votes=5)


def test_vision_votes_tolerate_jitter_and_failed_requests(mocker: Any) -> None:
    from openai import APITimeoutError

    module = _vision_module(
        mocker,
        [
            '{"found":true,"bbox":[606,773,664,846]}',
            '{"found":true,"bbox":[612,750,672,823]}',
            APITimeoutError(request=mocker.Mock()),
            '{"found":true,"bbox":[622,762,673,838]}',
            '{"found":true,"bbox":[141,83,219,188]}',
        ],
    )
    try:
        assert module._vision_bbox(b"jpeg", "the box", 640, 480) == [392, 366, 430, 402]
    finally:
        module.stop()


def test_vision_reports_model_error_when_every_request_fails(mocker: Any) -> None:
    from openai import APITimeoutError

    module = _vision_module(mocker, [APITimeoutError(request=mocker.Mock())] * 5)
    try:
        with pytest.raises(BridgeError) as error:
            module._vision_bbox(b"jpeg", "the box", 640, 480)
        assert error.value.code == "VISION_MODEL_ERROR"
    finally:
        module.stop()
