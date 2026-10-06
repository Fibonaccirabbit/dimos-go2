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

from dimos.mapping.voxels.module import VoxelMapTransformer
from dimos.memory.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_new_mapping_session_discards_old_height_but_preserves_current_obstacles():
    transformer = VoxelMapTransformer(voxel_size=0.1, carve_columns=False, device="CPU:0")
    old = Observation(ts=1.0, _data=PointCloud2.from_numpy(np.array([[0.01, 0.01, 0.51]])))
    current = Observation(
        ts=2.0, _data=PointCloud2.from_numpy(np.array([[0.01, 0.01, 0.01], [1.01, 0.01, 0.31]]))
    )
    accumulated = list(transformer(iter([old, current])))
    before, _ = accumulated[-1].data.as_numpy()
    np.testing.assert_allclose(before, [[0.05, 0.05, 0.05], [0.05, 0.05, 0.55], [1.05, 0.05, 0.35]])
    restarted = list(transformer(iter([current])))
    after, _ = restarted[-1].data.as_numpy()
    np.testing.assert_allclose(after, [[0.05, 0.05, 0.05], [1.05, 0.05, 0.35]])
