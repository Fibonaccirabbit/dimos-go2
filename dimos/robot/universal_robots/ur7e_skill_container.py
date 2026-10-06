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

"""Agent skills for the bounded UR7e ROS backend, without a servo coordinator."""

import json
import shlex
import subprocess
from threading import Event, RLock
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.hardware.manipulators.ur7e.adapter import UR7eROSAdapter
from dimos.hardware.manipulators.ur7e.policy import (
    GOAL_GRACE_S,
    MAX_DURATION_S,
    MIN_DURATION_S,
    BridgeError,
)


class UR7eSkillConfig(ModuleConfig):
    address: str
    remote_root: str
    control_path: str | None = None
    allow_motion: bool = False
    ros_setup: str = "/opt/ros/humble/setup.bash"
    controller: str = "scaled_joint_trajectory_controller"
    # Workstation plan files (relative to remote_root) for the demo whole-trajectory moves.
    hover_plan: str = "hover_plan_live.json"
    reset_plan: str = "hover_plan_reset.json"
    # Live cuRobo planning on the workstation (zero-shot hover above a located target).
    curobo_root: str = ""  # cuRobo checkout with its .venv on the workstation
    robot_config: str = "curobo/content/configs/robot/ur7e_factory_calibrated_closed.yml"
    camera_calibration: str = "static_candidates_refined_20261002.json"
    calibration_sample: str = "static_sample_20261002.npz"


class UR7eSkillContainer(Module):
    config: UR7eSkillConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._adapter: UR7eROSAdapter | None = None
        self._adapter_lock = RLock()
        self._shutdown = Event()
        self._return_plan: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        adapter = UR7eROSAdapter(
            address=self.config.address,
            remote_root=self.config.remote_root,
            control_path=self.config.control_path,
            allow_motion=self.config.allow_motion,
            controller=self.config.controller,
            ros_setup=self.config.ros_setup,
        )
        if not adapter.connect():
            super().stop()
            raise RuntimeError("UR7e bridge could not read live joint feedback")
        with self._adapter_lock:
            self._adapter = adapter
            self._shutdown.clear()

    @rpc
    def stop(self) -> None:
        self._shutdown.set()
        try:
            with self._adapter_lock:
                adapter, self._adapter = self._adapter, None
            if adapter is not None:
                adapter.disconnect()
        finally:
            super().stop()

    def _connected_adapter(self) -> UR7eROSAdapter:
        with self._adapter_lock:
            if self._adapter is None or self._shutdown.is_set():
                raise BridgeError("NOT_CONFIGURED", "UR7e module is not running")
            return self._adapter

    @skill
    def ur7e_status(self) -> SkillResult:
        """Read UR7e joint/TCP feedback and safety state. Check effort_unit: A is motor current, not Nm. Does not move the arm."""
        try:
            data = self._connected_adapter().snapshot(refresh=True)
        except BridgeError as exc:
            return SkillResult.fail(exc.code, str(exc))
        return SkillResult.ok("Live UR7e feedback read; no motion commanded", state=data)

    @skill
    def ur7e_preview_joint_delta(
        self,
        joint: int,
        delta_degrees: float,
        duration_s: float = MIN_DURATION_S,
    ) -> SkillResult:
        """Preview a small relative joint movement WITHOUT executing it. Not collision checking.

        Args:
            joint: Joint number, 1 through 6; 6 is wrist_3_joint.
            delta_degrees: Signed displacement in degrees, at most five degrees.
            duration_s: Nominal trajectory duration, 5 through 20 seconds.
        """
        try:
            data = self._connected_adapter().preview_joint_delta(joint, delta_degrees, duration_s)
        except BridgeError as exc:
            return SkillResult.fail(exc.code, str(exc))
        return SkillResult.ok(
            "Preview only; no trajectory sent and no collision check performed", preview=data
        )

    @skill(uses=["ur7e_motion"])
    def ur7e_move_joint_delta(
        self,
        joint: int,
        delta_degrees: float,
        duration_s: float = MIN_DURATION_S,
    ) -> SkillResult:
        """Execute ONE bounded joint test only in an operator-enabled motion session. Wait for measured result.

        Requires onsite clearance and supervision. This does NOT plan around
        obstacles. Never split larger moves into repeated five-degree calls.

        Args:
            joint: Joint number, 1 through 6.
            delta_degrees: Signed displacement in degrees, at most five degrees.
            duration_s: Nominal duration, 5 through 20 seconds.
        """
        try:
            adapter = self._connected_adapter()
            started = adapter.move_joint_delta(joint, delta_degrees, duration_s)
            motion_id = started["motion_id"]
            deadline = time.monotonic() + MAX_DURATION_S + GOAL_GRACE_S + 5
            while not self._shutdown.is_set():
                result = adapter.trajectory_result(motion_id)
                if result["done"]:
                    if result["success"]:
                        return SkillResult.ok(
                            "Controller succeeded and measured joint positions reached the target",
                            result=result,
                        )
                    return SkillResult(
                        success=False,
                        message="Trajectory did not complete successfully",
                        error_code="EXECUTION_FAILED",
                        metadata={"result": result},
                    )
                if time.monotonic() >= deadline:
                    stopped = adapter.write_stop()
                    return SkillResult.fail(
                        "EXECUTION_TIMEOUT",
                        f"Trajectory timed out; cancellation confirmed={stopped}",
                    )
                self._shutdown.wait(0.1)
            return SkillResult.fail(
                "INVALID_STATE",
                "Module stopped during execution; connection teardown cancels its own goal",
            )
        except BridgeError as exc:
            return SkillResult.fail(exc.code, str(exc))

    @skill
    def ur7e_stop(self) -> SkillResult:
        """Cancel this UR7e bridge's own unfinished trajectory. NOT an emergency stop or power-off."""
        try:
            stopped = self._connected_adapter().write_stop()
        except BridgeError as exc:
            return SkillResult.fail(exc.code, str(exc))
        if not stopped:
            return SkillResult.fail(
                "EXECUTION_FAILED", "Cancellation/standstill could not be confirmed"
            )
        return SkillResult.ok(
            "This connection has no unfinished trajectory; no power or safety mode changed"
        )

    def _run_plan(self, plan: str, prepare: str = "") -> SkillResult:
        if not self.config.allow_motion:
            return SkillResult.fail("READ_ONLY", "Motion is not enabled for this session")
        remote = prepare + (
            f"source {shlex.quote(self.config.ros_setup)} && "
            f"cd {shlex.quote(self.config.remote_root)} && "
            "python3 -m dimos.hardware.manipulators.ur7e.demo_hover_execute "
            f"--plan {shlex.quote(plan)} --controller {shlex.quote(self.config.controller)} "
            "--execute"
        )
        command = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]
        if self.config.control_path:
            command += ["-S", self.config.control_path]
        command += [self.config.address, "bash -lc " + shlex.quote(remote)]
        try:
            done = subprocess.run(command, capture_output=True, text=True, timeout=180, check=False)
        except subprocess.TimeoutExpired:
            self._connected_adapter().write_stop()
            return SkillResult.fail(
                "EXECUTION_TIMEOUT", "Planning and trajectory did not finish in 180 s"
            )
        lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
        result = json.loads(lines[-1]) if lines else {}
        if done.returncode != 0 or result.get("stage") != "FINISHED":
            reason = (done.stderr.strip().splitlines() or ["no output"])[-1]
            if "reach envelope" in reason:
                return SkillResult.fail(
                    "OUT_OF_REACH",
                    "Target is outside the arm's reach (needs 0.25-0.85 m horizontally from "
                    f"the base); nothing was sent. {reason}",
                )
            return SkillResult.fail("EXECUTION_FAILED", f"Not executed or failed: {reason}")
        if result.get("action_status") != 4 or result.get("final_error_deg", 99) > 0.5:
            return SkillResult(
                success=False,
                message="Controller did not confirm the measured goal",
                error_code="EXECUTION_FAILED",
                metadata={"result": result},
            )
        return SkillResult.ok("Trajectory finished; measured joints match the plan", result=result)

    @skill(uses=["ur7e_motion"])
    def ur7e_hover_above_white_box(self) -> SkillResult:
        """Move the gripper, pointing down, to 10 cm above the white box on the table.

        Executes a precomputed cuRobo whole trajectory (about 22 s). Only valid
        from the start pose; the executor refuses otherwise. Requires an
        operator-enabled motion session and onsite clearance.
        """
        result = self._run_plan(self.config.hover_plan)
        if result.success:
            self._return_plan = self.config.reset_plan
        return result

    @skill(uses=["ur7e_motion"])
    def ur7e_hover_above_point(
        self, point_color_optical_m: list[float], clearance_m: float = 0.1
    ) -> SkillResult:
        """Plan live with cuRobo from the current joints and move the gripper, pointing down, above a located target.

        Use the point returned by ur7e_locate_object. Plans with self-collision and
        a table model (no people or unknown objects), then executes slowly (~20-30 s).
        Requires an operator-enabled motion session and onsite clearance.

        Args:
            point_color_optical_m: Target top-surface point [x, y, z] in camera optical metres.
            clearance_m: Height of the gripper tip above the target, 0.05 to 0.3 m.
        """
        if not self.config.allow_motion:
            return SkillResult.fail("READ_ONLY", "Motion is not enabled for this session")
        if not self.config.curobo_root:
            return SkillResult.fail("NOT_CONFIGURED", "No workstation cuRobo checkout configured")
        point = [float(v) for v in point_color_optical_m]
        if len(point) != 3 or not 0.05 <= clearance_m <= 0.3:
            return SkillResult.fail("INVALID_TARGET", "Need [x, y, z] and clearance 0.05-0.3 m")
        try:
            joints = self._connected_adapter().snapshot(refresh=True)["positions"]
        except BridgeError as exc:
            return SkillResult.fail(exc.code, str(exc))
        root = self.config.remote_root
        plan = f"{root}/zs_plan.json"
        reverse = (
            "import json; d = json.load(open('zs_plan.json')); "
            "d['trajectory_joint_positions_rad'] = d['trajectory_joint_positions_rad'][::-1]; "
            "json.dump(d, open('zs_return.json', 'w'))"
        )
        prepare = (
            f"cd {shlex.quote(self.config.curobo_root)} && .venv/bin/python "
            f"{shlex.quote(root)}/dimos/hardware/manipulators/ur7e/demo_hover_plan.py "
            f"--sample {shlex.quote(root + '/' + self.config.calibration_sample)} "
            f"--calibration-candidates {shlex.quote(root + '/' + self.config.camera_calibration)} "
            f"--robot-config {shlex.quote(self.config.robot_config)} "
            f"--target-color-optical {' '.join(repr(v) for v in point)} "
            f"--start-joints {' '.join(repr(float(q)) for q in joints)} "
            f"--clearance {clearance_m!r} --output {shlex.quote(plan)} >/dev/null && "
            f"cd {shlex.quote(root)} && python3 -c {shlex.quote(reverse)} && "
        )
        result = self._run_plan("zs_plan.json", prepare)
        if result.success:
            self._return_plan = "zs_return.json"
        return result

    @skill(uses=["ur7e_motion"])
    def ur7e_return_to_start(self) -> SkillResult:
        """Return the arm to where the last hover move started, along the same path in reverse.

        Only valid right after a hover move; the executor refuses from any other pose.
        """
        result = self._run_plan(self._return_plan or self.config.reset_plan)
        if result.success:
            self._return_plan = None
        return result
