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

import os
from pathlib import Path
from threading import RLock
from typing import Any

from openai import OpenAI

from dimos.agents.llm_trace import tracing_http_client
from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs.Image import Image


class DeepSeekVlConfig(VlModelConfig):
    model_name: str = "deepseek-flash"
    base_url: str = "https://api.deepseek.com"
    api_key_env: str = "DEEPSEEK_API_KEY"
    timeout: float = 60.0
    trace_dir: Path | None = None


class DeepSeekVlModel(VlModel):
    """Responses API adapter for dimOS perception, using absolute pixel coordinates."""

    config: DeepSeekVlConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._client: OpenAI | None = None
        self._lock = RLock()

    def start(self) -> None:
        with self._lock:
            if self._client is not None:
                return
            api_key = os.environ.get(self.config.api_key_env)
            if not api_key:
                raise RuntimeError(
                    f"Missing API key environment variable {self.config.api_key_env!r}"
                )
            client_kwargs: dict[str, Any] = {}
            if self.config.trace_dir is not None:
                client_kwargs["http_client"] = tracing_http_client(self.config.trace_dir)
            self._client = OpenAI(
                api_key=api_key,
                base_url=self.config.base_url,
                timeout=self.config.timeout,
                max_retries=0,
                **client_kwargs,
            )

    def stop(self) -> None:
        with self._lock:
            if self._client is not None:
                self._client.close()
                self._client = None

    def query(self, image: Image, query: str, **kwargs: Any) -> str:
        prepared, _ = self._prepare_image(image)
        prompt = (
            f"The supplied image is {prepared.width} by {prepared.height} pixels. "
            "When asked for bounding boxes or points, use absolute image pixel coordinates, "
            "not normalized coordinates. Return only the requested format. "
            "Do not invent objects that are not visible.\n" + query
        )
        with self._lock:
            self.start()
            assert self._client is not None
            response = self._client.responses.create(
                model=self.config.model_name,
                input=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_image",
                                "image_url": f"data:image/jpeg;base64,{prepared.to_base64()}",
                            },
                            {"type": "input_text", "text": prompt},
                        ],
                    }
                ],
                **kwargs,
            )
        if not response.output_text.strip():
            raise RuntimeError("DeepSeek vision returned an empty answer")
        return response.output_text
