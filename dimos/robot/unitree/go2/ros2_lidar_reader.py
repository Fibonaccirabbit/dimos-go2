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
"""Executed over SSH on Go2 EDU. Only subscribes; never publishes to ROS.

Stdout is a length-bounded JSON header followed by packed little-endian XYZ.
No robot files, services, motion modes, or persistent settings are changed.
"""

import json
import sys
import time
from typing import Any

import numpy as np


def extract_xyz(msg: Any) -> np.ndarray:
    """Read padded ROS rows, endianness and XYZ offsets; drop Go2 zero padding."""
    fields = {field.name: field for field in msg.fields}
    if any(fields[axis].datatype != 7 for axis in ("x", "y", "z")):
        raise ValueError("Bridge requires float32 XYZ fields")
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [">f4" if msg.is_bigendian else "<f4"] * 3,
            "offsets": [fields[axis].offset for axis in ("x", "y", "z")],
            "itemsize": msg.point_step,
        }
    )
    records = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    )
    points = np.column_stack([records[axis].ravel() for axis in ("x", "y", "z")])
    # EDU cloud_deskewed currently appends 10,000 zero XYZ records per scan.
    # They are placeholders, not measurements of a floor at the odom origin.
    valid = np.isfinite(points).all(axis=1) & np.any(points != 0, axis=1)
    return points[valid].astype("<f4", copy=False)


def main() -> None:
    # ROS 2 is available on the EDU, not required on the Mac running dimOS.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    topic, expected_frame, rate = sys.argv[1:]
    period = 1.0 / float(rate)
    previous = 0.0
    rclpy.init()
    node = rclpy.create_node("dimos_readonly_lidar_bridge")

    def receive(msg: PointCloud2) -> None:
        nonlocal previous
        now = time.monotonic()
        if now - previous < period:
            return
        if msg.header.frame_id != expected_frame:
            raise ValueError(f"Expected {expected_frame}, received {msg.header.frame_id}")
        points = extract_xyz(msg)
        if not len(points):
            return
        payload = points.tobytes()
        header = {
            "version": 1,
            "frame": msg.header.frame_id,
            "points": len(points),
            "bytes": len(payload),
            "sensor_ts": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        }
        sys.stdout.buffer.write(json.dumps(header).encode() + b"\n" + payload)
        sys.stdout.buffer.flush()
        previous = now

    node.create_subscription(PointCloud2, topic, receive, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
