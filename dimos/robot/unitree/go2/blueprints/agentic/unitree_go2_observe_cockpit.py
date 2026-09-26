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

"""Physical Go2 mapping and planning preview; no robot motion capabilities."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.go2.planning_preview import Go2PlanningPreview
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.ros2_lidar import Go2Ros2Lidar
from dimos.web.cockpit import Channel, Chat, Col, Map2D, Map3D, Row, Stats, Video, cockpit

unitree_go2_observe_cockpit = autoconnect(
    GO2Connection.blueprint(read_only=True, lidar=False).remappings(
        [(GO2Connection, "lidar", "webrtc_lidar_unused")]
    ),
    Go2Ros2Lidar.blueprint(),
    VoxelGridMapper.blueprint(emit_every=5, carve_columns=False),
    CostMapper.blueprint(),
    Go2PlanningPreview.blueprint(),
    ObserveSkill.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(),
    cockpit(
        channels=[
            Channel(
                "navigation_costmap",
                OccupancyGrid,
                encoding="costmap.zlib.v1",
                delivery="latest",
                max_hz=5.0,
            )
        ],
        layout=Row(
            Col(
                Video("color_image", title="Real Go2 camera · sensors only"),
                Map3D(title="Real ROS 2 lidar · dimOS voxel map"),
            ),
            Col(
                Map2D(path="path", title="Real costmap · path preview only"),
                Chat(title="Observe / plan only · movement locked"),
            ),
            shares=[2, 1],
        ),
        pages=[
            Map3D(cloud="lidar", title="Raw deskewed lidar"),
            Map2D(costmap="navigation_costmap", path="path", title="Inflated known-space preview"),
            Video("color_image", title="Real Go2 camera"),
            Stats(),
        ],
    ),
).global_config(n_workers=9, robot_model="unitree_go2")
