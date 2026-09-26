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

from dimos.perception.experimental.image_embedding import ImageEmbeddingProvider


@pytest.fixture
def uninitialized_provider(mocker):
    mocker.patch.object(ImageEmbeddingProvider, "_initialize_model")
    return ImageEmbeddingProvider()


def test_missing_image_model_does_not_return_random_memory(uninitialized_provider):
    with pytest.raises(RuntimeError, match="not initialized"):
        uninitialized_provider.get_embedding(np.zeros((2, 2, 3), dtype=np.uint8))


def test_missing_text_model_does_not_return_random_memory(uninitialized_provider):
    with pytest.raises(RuntimeError, match="not initialized"):
        uninitialized_provider.get_text_embedding("kitchen")
