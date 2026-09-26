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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental import spatial_perception
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.perception.experimental.visual_memory import VisualMemory


@pytest.fixture
def memory(mocker, tmp_path):
    mocker.patch.object(spatial_perception, "ImageEmbeddingProvider")
    visual = VisualMemory(output_dir=str(tmp_path))
    visual.add("observed-sofa", np.full((2, 2, 3), 128, dtype=np.uint8))
    path = str(tmp_path / "memory.pkl")
    module = SpatialMemory(
        db_path=None,
        chroma_client=mocker.MagicMock(),
        visual_memory=visual,
        visual_memory_path=path,
        output_dir=str(tmp_path),
    )
    yield module, path
    module.stop()


def test_repeated_teardown_preserves_observed_images(memory):
    module, path = memory
    module.stop()
    module.stop()
    restored = VisualMemory.load(path)
    assert restored.count() == 1
    assert restored.contains("observed-sofa")


def test_semantic_query_excludes_legacy_vectors_without_images(memory):
    module, _ = memory
    module.embedding_provider.get_text_embedding.return_value = np.ones(2)
    module.vector_db.image_collection.query.return_value = {
        "ids": [["lost-image", "observed-sofa"]],
        "metadatas": [[{"pos_x": 1.0}, {"pos_x": 2.0}]],
        "distances": [[0.1, 0.2]],
    }
    results = module.query_by_text("sofa")
    assert results == [{"id": "observed-sofa", "metadata": [{"pos_x": 2.0}], "distance": 0.2}]


@pytest.mark.parametrize(
    ("previous_yaw", "current_yaw", "expected_count"),
    [(0.0, 0.6, 1), (0.0, 0.1, 0), (np.pi - 0.05, -np.pi + 0.05, 0)],
)
def test_sampling_records_turns_but_not_stationary_duplicates(
    memory, mocker, previous_yaw, current_yaw, expected_count
):
    module, _ = memory
    module.config.min_rotation_threshold = 0.35
    module.min_distance_threshold = 0.35
    module.last_position = Vector3(0, 0, 0)
    module.last_yaw = previous_yaw
    module._latest_video_frame = np.full((2, 2, 3), 128, dtype=np.uint8)
    transform = mocker.MagicMock()
    transform.to_pose.return_value = PoseStamped(position=Vector3(0, 0, 0))
    transform.rotation.to_euler.return_value = Vector3(0, 0, current_yaw)
    tf_buffer = mocker.MagicMock()
    tf_buffer.get.return_value = transform
    mocker.patch.object(
        SpatialMemory, "tfbuffer", new_callable=mocker.PropertyMock, return_value=tf_buffer
    )
    module.embedding_provider.get_embedding.return_value = np.ones(2)
    module._process_frame()
    assert module.stored_frame_count == expected_count
    assert module.vector_db.image_collection.add.call_count == expected_count
