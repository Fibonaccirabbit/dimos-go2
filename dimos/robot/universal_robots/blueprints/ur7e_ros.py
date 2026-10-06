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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.universal_robots.system_prompt import UR7E_SYSTEM_PROMPT
from dimos.robot.universal_robots.ur7e_skill_container import UR7eSkillContainer

_ur7e_skills = UR7eSkillContainer.blueprint(rpc_timeouts={"ur7e_move_joint_delta": 40.0})

ur7e_ros = autoconnect(_ur7e_skills, McpServer.blueprint())
ur7e_ros_agentic = autoconnect(
    ur7e_ros,
    McpClient.blueprint(system_prompt=UR7E_SYSTEM_PROMPT, excluded_tools=("agent_send",)),
)
