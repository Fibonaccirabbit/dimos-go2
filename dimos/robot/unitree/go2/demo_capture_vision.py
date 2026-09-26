# Copyright 2026 Fibonaccirabbit
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

"""Seed a two-object visual demo and capture the live DimSim RGB/odom streams."""

import argparse
import json
from pathlib import Path
import time

import lcm

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.dimsim.scene_client import SceneClient


def capture(output: Path, seconds: float) -> None:
    bus = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
    images: list[Image] = []
    poses: list[PoseStamped] = []
    subs = [
        bus.subscribe(
            "/color_image#sensor_msgs.Image", lambda _, data: images.append(Image.lcm_decode(data))
        ),
        bus.subscribe(
            "/odom#geometry_msgs.PoseStamped",
            lambda _, data: poses.append(PoseStamped.lcm_decode(data)),
        ),
    ]
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            bus.handle_timeout(100)
    finally:
        for sub in subs:
            bus.unsubscribe(sub)
    if not images or not poses:
        raise RuntimeError(f"Missing live sensor data: RGB={len(images)}, odom={len(poses)}")
    output.mkdir(parents=True, exist_ok=True)
    frame = images[-1]
    if not frame.save(str(output / "camera.jpg")):
        raise RuntimeError("Could not save camera image")
    pose = poses[-1]
    report = {
        "rgb_frames": len(images),
        "rgb_hz": round(len(images) / seconds, 1),
        "shape": frame.shape,
        "frame_id": frame.frame_id,
        "frame_age_s": round(time.time() - frame.ts, 3),
        "odom": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
        "image": str((output / "camera.jpg").resolve()),
    }
    (output / "sensors.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False))


def seed(red_side: str) -> None:
    client = SceneClient()
    client.start()
    try:
        # Three.js +x is ROS +y (left when the robot is facing ROS +x).
        red_x, blue_x = (3.2, 0.8) if red_side == "left" else (0.8, 3.2)
        print(
            client.exec(f"""
for (const name of ['vision_demo_red', 'vision_demo_blue']) {{
    const old = scene.getObjectByName(name);
    if (old) {{ scene.remove(old); old.geometry.dispose(); old.material.dispose(); }}
}}
const red = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.7, 0.7),
    new THREE.MeshStandardMaterial({{color: 0xff2020, roughness: 0.8}}));
red.name = 'vision_demo_red'; red.position.set({red_x}, 0.35, 6); scene.add(red);
const blue = new THREE.Mesh(new THREE.SphereGeometry(0.4, 32, 16),
    new THREE.MeshStandardMaterial({{color: 0x2050ff, roughness: 0.8}}));
blue.name = 'vision_demo_blue'; blue.position.set({blue_x}, 0.4, 6); scene.add(blue);
return {{redSide: {json.dumps(red_side)}, agentPosition: agent.getPosition()}};
""")
        )
    finally:
        client.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", choices=("left", "right"))
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.seed:
        seed(args.seed)
    capture(args.output, args.seconds)


if __name__ == "__main__":
    main()
