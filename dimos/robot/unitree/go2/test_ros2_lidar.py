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

import io
import json
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.robot.unitree.go2.ros2_lidar import Go2Ros2Lidar, read_cloud_packet
from dimos.robot.unitree.go2.ros2_lidar_reader import extract_xyz


def packet(xyz, **header_changes):
    data = np.asarray(xyz, dtype="<f4").reshape(-1, 3).tobytes()
    header = {"version": 1, "frame": "odom", "points": len(data) // 12, "bytes": len(data)}
    header.update(header_changes)
    return json.dumps(header).encode() + b"\n" + data


def test_packet_preserves_coordinates_with_explicit_odom_world_alias():
    stream = io.BytesIO(packet([[2, -1, 0.5], [-3, 4, -0.2]]))
    cloud = read_cloud_packet(stream, "odom", "world")
    points, _ = cloud.as_numpy()
    np.testing.assert_allclose(points, [[2, -1, 0.5], [-3, 4, -0.2]])
    assert cloud.frame_id == "world"


@pytest.mark.parametrize(
    "changes",
    [
        {"frame": "base_link"},
        {"points": 600_000},
        {"bytes": 11},
        {"version": 2},
    ],
)
def test_packet_rejects_frame_or_size_mismatch(changes):
    with pytest.raises(ValueError, match="Invalid pointcloud"):
        read_cloud_packet(io.BytesIO(packet([[1, 2, 3]], **changes)), "odom", "world")


def test_packet_rejects_truncated_payload():
    with pytest.raises(EOFError, match="Truncated"):
        read_cloud_packet(io.BytesIO(packet([[1, 2, 3]])[:-1]), "odom", "world")


def test_packet_rejects_nonfinite_coordinates():
    with pytest.raises(ValueError, match="Non-finite"):
        read_cloud_packet(io.BytesIO(packet([[1, np.nan, 3]])), "odom", "world")


@pytest.fixture
def bridge():
    module = Go2Ros2Lidar()
    yield module
    module.stop()


def test_bridge_status_does_not_claim_data_before_start(bridge):
    status = bridge.pointcloud_status()
    assert status["motion_enabled"] is False
    assert status["frames"] == 0
    assert status["age_seconds"] is None


@pytest.mark.parametrize("endian", ["<", ">"])
def test_ros_reader_honors_padded_rows_and_filters_zero_placeholder(endian):
    # Two one-point rows with row padding and XYZ field offsets.
    data = struct.pack(endian + "ffff", 99, 1, 2, 3) + b"padding!"
    data += struct.pack(endian + "ffff", 99, 0, 0, 0) + b"padding!"
    msg = SimpleNamespace(
        fields=[
            SimpleNamespace(name=axis, offset=offset, datatype=7)
            for axis, offset in zip(["x", "y", "z"], [4, 8, 12], strict=True)
        ],
        is_bigendian=endian == ">",
        point_step=16,
        row_step=24,
        height=2,
        width=1,
        data=data,
    )
    np.testing.assert_array_equal(extract_xyz(msg), [[1, 2, 3]])
