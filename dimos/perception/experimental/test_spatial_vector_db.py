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

from dimos.perception.experimental.spatial_vector_db import SpatialVectorDB


@pytest.fixture
def database(mocker):
    client = mocker.MagicMock()
    collection = client.get_or_create_collection.return_value
    return SpatialVectorDB(chroma_client=client), collection


def test_retrieval_preserves_each_ranked_match_and_its_pose(database):
    db, collection = database
    collection.query.return_value = {
        "ids": [["kitchen-frame", "sofa-frame"]],
        "metadatas": [[{"pos_x": 3.0}, {"pos_x": 7.0}]],
        "distances": [[0.2, 0.7]],
    }
    assert db.query_by_embedding(np.array([1.0, 0.0]), limit=2) == [
        {"id": "kitchen-frame", "metadata": [{"pos_x": 3.0}], "distance": 0.2},
        {"id": "sofa-frame", "metadata": [{"pos_x": 7.0}], "distance": 0.7},
    ]
    collection.query.assert_called_once_with(query_embeddings=[[1.0, 0.0]], n_results=2)


def test_empty_retrieval_is_not_a_location(database):
    db, collection = database
    collection.query.return_value = {"ids": [[]], "metadatas": [[]], "distances": [[]]}
    assert db.query_by_embedding(np.array([1.0])) == []


def test_empty_tag_collection_does_not_download_a_text_model(database):
    db, collection = database
    collection.count.return_value = 0
    assert db.query_tagged_location("kitchen") == (None, 0.0)
    collection.query.assert_not_called()
