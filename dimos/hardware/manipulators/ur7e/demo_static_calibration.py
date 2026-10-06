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

"""Offline static ICP candidates on the workstation. NO ROS, motion, or TF publication.

Requires its existing cuRobo environment. A fit is a candidate, not a validated
hand-eye calibration. Segmentation is operator-selected and exported for audit.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

from curobo.kinematics import Kinematics, KinematicsCfg  # type: ignore[import-not-found]
from curobo.perception import (  # type: ignore[import-not-found]
    DetectorCfg,
    PoseDetector,
    RobotMesh,
    SDFDetectorCfg,
    SDFPoseDetector,
)
from curobo.types import DeviceCfg  # type: ignore[import-not-found]
import numpy as np
from scipy.spatial import cKDTree
import torch
import yaml  # type: ignore[import-untyped]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", required=True)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    started = time.monotonic()
    sample = np.load(args.sample)
    observed = sample["robot_points_color_optical_m"]
    if observed.shape[0] < 500:
        raise ValueError("Insufficient segmented robot points")
    config = yaml.safe_load(Path(args.robot_config).read_text())
    config["kinematics"]["mesh_link_names"] = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    device = DeviceCfg(device=args.device)
    kin = Kinematics(
        KinematicsCfg.from_robot_yaml_file(
            config, device_cfg=device, load_tool_frames_with_mesh=True
        )
    )
    names = list(kin.joint_names)
    q = torch.as_tensor(sample["joint_positions_rad"], device=device.device, dtype=device.dtype)
    if q.numel() != len(names):
        raise ValueError(f"Robot joint mismatch: {names}")
    mesh = RobotMesh.from_kinematics(kin, device=args.device)
    mesh.update(q)
    candidates: list[dict[str, Any]] = []
    for seed in (17, 29):
        torch.manual_seed(seed)
        detector = PoseDetector(
            mesh,
            DetectorCfg(
                n_mesh_points_coarse=1000,
                n_observed_points_coarse=3000,
                n_rotation_samples=64,
                n_iterations_coarse=35,
                n_mesh_points_fine=6000,
                n_observed_points_fine=5000,
                n_iterations_fine=40,
                distance_threshold_fine=0.04,
                device_cfg=device,
            ),
        )
        result = detector.detect_from_points(
            torch.as_tensor(observed, device=device.device, dtype=device.dtype), q
        )
        camera_from_base = result.pose.get_matrix().squeeze().detach().cpu().numpy()
        model_points, _ = mesh.sample_surface_points(30000)
        model = (
            model_points.detach().cpu().numpy() @ camera_from_base[:3, :3].T
            + camera_from_base[:3, 3]
        )
        distances, _ = cKDTree(model).query(observed)
        retained = observed[distances < 0.025]
        sdf = SDFPoseDetector(
            mesh,
            SDFDetectorCfg(
                max_iterations=30,
                inner_iterations=5,
                n_points=4000,
                distance_threshold=0.03,
                huber_delta=0.01,
                device_cfg=device,
            ),
        )
        refined = sdf.detect_from_points(
            torch.as_tensor(retained[::2].copy(), device=device.device, dtype=device.dtype),
            q,
            initial_pose=result.pose,
        )
        camera_from_base = refined.pose.get_matrix().squeeze().detach().cpu().numpy()
        model_points, _ = mesh.sample_surface_points(60000)
        model = (
            model_points.detach().cpu().numpy() @ camera_from_base[:3, :3].T
            + camera_from_base[:3, 3]
        )
        holdout_distances, _ = cKDTree(model).query(retained[1::2])
        candidates.append(
            {
                "seed": seed,
                "color_optical_from_base_link": camera_from_base.tolist(),
                "base_link_from_color_optical": np.linalg.inv(camera_from_base).tolist(),
                "icp_alignment_error_m": float(result.alignment_error),
                "observed_to_mesh_median_m": float(np.median(distances)),
                "observed_to_mesh_p90_m": float(np.percentile(distances, 90)),
                "observed_fraction_within_2cm": float(np.mean(distances < 0.02)),
                "sdf_alignment_error_m": float(refined.alignment_error),
                "retained_points": len(retained),
                "trimmed_holdout_median_m": float(np.median(holdout_distances)),
                "trimmed_holdout_p90_m": float(np.percentile(holdout_distances, 90)),
            }
        )
    output = {
        "validation": "UNVALIDATED_STATIC_CANDIDATES",
        "motion_allowed": False,
        "robot_config": args.robot_config,
        "joint_names": names,
        "segmented_points": len(observed),
        "elapsed_s": time.monotonic() - started,
        "candidates": candidates,
    }
    Path(args.output).write_text(json.dumps(output, indent=2))
    print(json.dumps(output))


if __name__ == "__main__":
    main()
