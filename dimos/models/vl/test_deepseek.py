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

import numpy as np
import pytest

from dimos.models.vl import deepseek
from dimos.models.vl.create import create
from dimos.models.vl.deepseek import DeepSeekVlModel
from dimos.msgs.sensor_msgs.Image import Image


@pytest.fixture
def image():
    return Image.from_numpy(np.full((24, 40, 3), 128, dtype=np.uint8))


@pytest.fixture
def model_client(mocker, monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "test-only-key")
    client = mocker.MagicMock()
    client.responses.create.return_value.output_text = '[["sofa", 2, 3, 20, 18]]'
    mocker.patch.object(deepseek, "OpenAI", return_value=client)
    model = DeepSeekVlModel()
    yield model, client
    model.stop()


def test_query_delivers_image_and_pixel_coordinate_contract(model_client, image):
    model, client = model_client
    assert model.query(image, "Find the sofa") == '[["sofa", 2, 3, 20, 18]]'
    request = client.responses.create.call_args.kwargs
    content = request["input"][0]["content"]
    assert content[0]["type"] == "input_image"
    assert content[0]["image_url"].startswith("data:image/jpeg;base64,")
    assert "40 by 24 pixels" in content[1]["text"]
    assert "absolute image pixel coordinates" in content[1]["text"]
    assert content[1]["text"].endswith("Find the sofa")


def test_detection_result_preserves_absolute_pixels(model_client, image):
    model, _ = model_client
    result = model.query_detections(image, "sofa")
    assert [(d.name, d.bbox) for d in result.detections] == [("sofa", (2, 3, 20, 18))]


def test_query_rejects_empty_provider_answer(model_client, image):
    model, client = model_client
    client.responses.create.return_value.output_text = "  "
    with pytest.raises(RuntimeError, match="empty answer"):
        model.query(image, "Describe")


def test_missing_key_fails_before_client_creation(mocker, monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    constructor = mocker.patch.object(deepseek, "OpenAI")
    with pytest.raises(RuntimeError, match="Missing API key"), DeepSeekVlModel():
        pass
    constructor.assert_not_called()


def test_stop_closes_owned_client(model_client):
    model, client = model_client
    model.start()
    model.stop()
    client.close.assert_called_once_with()


def test_factory_selects_deepseek_without_network():
    model = create("deepseek")
    assert isinstance(model, DeepSeekVlModel)
