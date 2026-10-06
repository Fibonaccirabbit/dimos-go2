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

"""Read-only adapter smoke or a dimOS/MCP session. Never moves at startup."""

import argparse
import json
from typing import Any

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.ur7e.adapter import UR7eROSAdapter
from dimos.robot.universal_robots.camera_module import UR7eCameraModule
from dimos.robot.universal_robots.system_prompt import UR7E_SYSTEM_PROMPT
from dimos.robot.universal_robots.ur7e_skill_container import UR7eSkillContainer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", required=True, help="SSH host or user@host; no password")
    parser.add_argument("--remote-root", required=True)
    parser.add_argument("--control-path")
    parser.add_argument("--mcp", action="store_true")
    parser.add_argument("--agent", action="store_true")
    parser.add_argument("--model")
    parser.add_argument("--model-base-url")
    parser.add_argument("--model-api-key-env")
    parser.add_argument(
        "--model-use-responses-api", action=argparse.BooleanOptionalAction, default=None
    )
    parser.add_argument("--mcp-port", type=int, default=global_config.mcp_port)
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--camera-port", type=int, help="Optional loopback RGB-D preview port")
    parser.add_argument("--camera-namespace", default="/ur7e_camera")
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="Operator-enabled bounded tests; not collision planning",
    )
    args = parser.parse_args()
    options = {
        "address": args.address,
        "remote_root": args.remote_root,
        "control_path": args.control_path,
        "allow_motion": args.allow_motion,
    }
    if not args.mcp and not args.agent:
        adapter = UR7eROSAdapter(**options)
        try:
            if not adapter.connect():
                raise SystemExit("UR7e read-only smoke failed")
            print(
                json.dumps(
                    {
                        "state": adapter.snapshot(refresh=True),
                        "preview": adapter.preview_joint_delta(6, 0.5),
                        "executed": False,
                    },
                    indent=2,
                )
            )
        finally:
            adapter.disconnect()
        return
    blueprint = autoconnect(
        UR7eSkillContainer.blueprint(**options, rpc_timeouts={"ur7e_move_joint_delta": 40.0}),
        McpServer.blueprint(),
    )
    if args.camera:
        blueprint = autoconnect(
            blueprint,
            UR7eCameraModule.blueprint(
                address=args.address,
                remote_root=args.remote_root,
                control_path=args.control_path,
                namespace=args.camera_namespace,
                viewer_port=args.camera_port,
            ),
        )
    if args.agent:
        agent_options: dict[str, Any] = {
            "system_prompt": UR7E_SYSTEM_PROMPT,
            "excluded_tools": ("agent_send",),
            "mcp_server_url": f"http://127.0.0.1:{args.mcp_port}/mcp",
        }
        if args.model:
            agent_options["model"] = args.model
        if args.model_base_url:
            agent_options["model_base_url"] = args.model_base_url
        if args.model_use_responses_api is not None:
            agent_options["model_use_responses_api"] = args.model_use_responses_api
        if args.model_api_key_env:
            agent_options["model_api_key_env"] = args.model_api_key_env
        blueprint = autoconnect(blueprint, McpClient.blueprint(**agent_options))
    blueprint = blueprint.global_config(
        mcp_port=args.mcp_port,
        listen_host="127.0.0.1",
        viewer="none",
    )
    ModuleCoordinator.build(blueprint).loop()


if __name__ == "__main__":
    main()
