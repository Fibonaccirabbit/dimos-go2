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

"""UR7e cockpit: RGB camera left, Agent chat right, plus camera/stats pages.

Separate file on purpose: cockpit() needs the [web] extra at import time, and
`ur7e-ros-agentic` itself must stay importable without it. Connection options
(address, remote_root, allow_motion, ...) come from module CLI arguments.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.universal_robots.camera_module import UR7eCameraModule
from dimos.robot.universal_robots.system_prompt import UR7E_SYSTEM_PROMPT
from dimos.robot.universal_robots.ur7e_skill_container import UR7eSkillContainer
from dimos.web.cockpit import Channel, Chat, Col, Row, Stats, Video, cockpit

_MOTION_TIMEOUT_S = 100.0

ur7e_ros_agentic_cockpit = autoconnect(
    UR7eSkillContainer.blueprint(
        rpc_timeouts={
            "ur7e_move_joint_delta": 40.0,
            "ur7e_hover_above_white_box": _MOTION_TIMEOUT_S,
            "ur7e_hover_above_point": 2 * _MOTION_TIMEOUT_S,
            "ur7e_return_to_start": _MOTION_TIMEOUT_S,
        }
    ),
    UR7eCameraModule.blueprint(rpc_timeouts={"ur7e_locate_object": 120.0}),
    ObserveSkill.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=UR7E_SYSTEM_PROMPT, excluded_tools=("agent_send",)),
    cockpit(
        channels=[
            Channel(
                "target_image", Image, encoding="jpeg.v1", delivery="latest", params={"quality": 75}
            )
        ],
        layout=Row(
            Video("color_image", title="UR7e workspace camera (Orbbec Gemini 335L)"),
            Col(
                Video("target_image", title="Last located target"),
                Chat(title="UR7e Agent"),
                shares=[1, 2],
            ),
            shares=[3, 2],
        ),
        pages=[Video("color_image", title="UR7e workspace camera"), Stats()],
    ),
).global_config(n_workers=6)
