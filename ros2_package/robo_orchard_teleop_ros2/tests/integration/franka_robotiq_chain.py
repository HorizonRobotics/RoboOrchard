# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

"""Real ROS chain regression with a synthetic arm and SDK fake gripper.

Run directly in an isolated ROS domain; see the teleop README.
"""

import copy
import os
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path
from tempfile import TemporaryDirectory

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robo_orchard_control_manager_ros2.node import ControlManagerNode
from robo_orchard_pico_msg_ros2.msg import VRState
from robo_orchard_teleop_ros2.engagement import EngagementNode
from robo_orchard_teleop_ros2.robot.franka.pico_vr import (
    FrankaPicoVRTeleOpNode,
)
from robo_orchard_teleop_ros2.robot.robotiq.pico_vr import (
    RobotiqPicoVRTeleOpNode,
)


def stop_process(process: subprocess.Popen) -> None:
    """Stop only a child process started by this test."""
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main() -> None:
    """Exercise the shipped two-route wiring without physical hardware."""
    with TemporaryDirectory() as directory, ExitStack() as cleanup:
        assert os.environ.get("ROS_DOMAIN_ID") not in (None, "", "0"), (
            "Choose an unused nonzero ROS_DOMAIN_ID for this test."
        )
        assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
        config_dir = Path(__file__).resolve().parents[2] / "config"
        manager = yaml.safe_load(
            (config_dir / "franka_robotiq_manager.example.yaml").read_text()
        )
        manager["channels"][0]["output_topic"] = "/smoke/arm_driver_commands"
        manager["reset_services"] = []
        manager_path = Path(directory) / "manager.yaml"
        manager_path.write_text(yaml.safe_dump(manager))
        urdf = Path(directory) / "arm.urdf"
        # Synthetic seven-axis chain: this is not a physical Franka/tool model.
        links = "".join(f'<link name="fr3_link{i}"/>' for i in range(9))
        joints = "".join(
            f'<joint name="fr3_joint{i}" type="revolute">'
            f'<parent link="fr3_link{i - 1}"/><child link="fr3_link{i}"/>'
            f'<origin xyz="0 0 0.1"/><axis xyz="{axis}"/>'
            '<limit lower="-2" upper="2" effort="1" velocity="1"/></joint>'
            for i, axis in enumerate(
                (
                    "0 0 1",
                    "0 1 0",
                    "1 0 0",
                    "0 1 0",
                    "1 0 0",
                    "0 1 0",
                    "0 0 1",
                ),
                start=1,
            )
        )
        urdf.write_text(
            '<robot name="synthetic_fr3">'
            + links
            + joints
            + '<joint name="flange" type="fixed"><parent link="fr3_link7"/>'
            '<child link="fr3_link8"/></joint></robot>'
        )
        driver = (
            Path(get_package_prefix("robo_orchard_robotiq_ros2"))
            / "lib/robo_orchard_robotiq_ros2/robotiq_driver_node"
        )
        process = subprocess.Popen(
            [
                str(driver),
                "--ros-args",
                "-r",
                "__ns:=/robotiq",
                "-p",
                "use_fake:=true",
            ]
        )
        cleanup.callback(stop_process, process)
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"control_manager_node:config_file:={manager_path}",
            ]
        )
        cleanup.callback(rclpy.shutdown)
        manager_node = ControlManagerNode()
        cleanup.callback(manager_node.destroy_node)
        arm = FrankaPicoVRTeleOpNode(
            namespace="/smoke/arm",
            cli_args=["--ros-args", "-r", "vr_state:=/pico_bridge/vr_state"],
            parameter_overrides=[
                Parameter("urdf_path", value=str(urdf)),
                Parameter("publish_activation", value=True),
                Parameter(
                    "joint_command_topic", value="/franka_teleop/candidate"
                ),
                Parameter(
                    "recapture_service",
                    value="/franka_teleop/recapture_baseline",
                ),
                Parameter("joint_state_topic", value="/smoke/joints"),
                Parameter("ee_pose_topic", value="/smoke/pose"),
                Parameter("robot_state_topic", value="/smoke/robot_state"),
                Parameter("control_frequency_hz", value=30.0),
            ],
        )
        cleanup.callback(arm.destroy_node)
        hand = RobotiqPicoVRTeleOpNode(
            parameter_overrides=[
                Parameter("match_dwell_s", value=0.1),
                Parameter("match_tolerance_m", value=0.001),
            ]
        )
        cleanup.callback(hand.destroy_node)
        gate = EngagementNode(
            parameter_overrides=[
                Parameter(
                    "config_file",
                    value=str(config_dir / "franka_robotiq_engagement.yaml"),
                ),
            ]
        )
        cleanup.callback(gate.destroy_node)
        harness = Node("engagement_chain_validation")
        cleanup.callback(harness.destroy_node)
        executor = SingleThreadedExecutor()
        cleanup.callback(executor.shutdown)
        for node in (arm, hand, gate, manager_node, harness):
            executor.add_node(node)
        vr_pub = harness.create_publisher(VRState, "/pico_bridge/vr_state", 1)
        joint_pub = harness.create_publisher(JointState, "/smoke/joints", 1)
        pose_pub = harness.create_publisher(PoseStamped, "/smoke/pose", 1)
        robot_pub = harness.create_publisher(
            FrankaRobotState, "/smoke/robot_state", 1
        )
        old_pub = harness.create_publisher(
            JointState, "/franka_teleop/candidate", 1
        )
        arm_outputs, hand_outputs, candidates, feedback = [], [], [], []
        harness.create_subscription(
            JointState, "/smoke/arm_driver_commands", arm_outputs.append, 10
        )
        harness.create_subscription(
            JointState, "/robotiq/joint_commands", hand_outputs.append, 10
        )
        harness.create_subscription(
            JointState, "/franka_teleop/candidate", candidates.append, 10
        )
        harness.create_subscription(
            JointState, "/robotiq/joint_states", feedback.append, 10
        )
        activate = harness.create_client(Trigger, "/robotiq/activate")
        takeover = harness.create_client(Trigger, "/robot/control/takeover")
        stop = harness.create_client(Trigger, "/robot/control/stop")
        grip, trigger, live = 0.0, 0.5, True

        def feed() -> None:
            if not live:
                return
            stamp = harness.get_clock().now().to_msg()
            vr = VRState()
            # Device and ROS clocks need not agree; only VR is offset.
            vr.header.stamp = copy.deepcopy(stamp)
            vr.header.stamp.sec += 130
            vr.right_controller.status = 1
            vr.right_controller.pose.orientation.w = 1.0
            vr.right_controller.gripper = grip
            vr.right_controller.trigger = trigger
            vr_pub.publish(vr)
            joints = JointState()
            joints.header.stamp = stamp
            joints.name = [f"fr3_joint{i}" for i in range(1, 8)]
            joints.position = [0.0] * 7
            joint_pub.publish(joints)
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.pose.orientation.w = 1.0
            pose.pose.position.z = 0.7
            pose_pub.publish(pose)
            robot = FrankaRobotState()
            robot.f_t_ee.pose.orientation.w = 1.0
            robot_pub.publish(robot)

        harness.create_timer(0.02, feed)

        def spin(seconds: float) -> None:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)

        def call(client) -> None:
            assert client.wait_for_service(timeout_sec=5), client.srv_name
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert future.done() and future.result().success

        spin(0.4)
        call(activate)
        call(takeover)
        spin(0.3)
        grip = 1.0
        spin(1.3)
        assert candidates and gate._phase == "matching", (
            gate._phase,
            gate._reason,
        )
        assert not arm_outputs and not hand_outputs
        old = copy.deepcopy(candidates[-1])
        trigger = max(0.0, min(1.0, 1.0 - feedback[-1].position[0] / 0.085))
        spin(0.6)
        assert gate._phase == "active", (gate._phase, gate._reason)
        assert arm_outputs and hand_outputs
        old.position = [1.9] * 7
        old_pub.publish(old)
        spin(0.1)
        assert gate._phase == "active"
        assert all(message.position != old.position for message in arm_outputs)
        trigger = 0.7
        spin(0.15)
        assert gate._phase == "active", gate._reason
        grip = 0.0
        spin(0.2)
        counts = len(arm_outputs), len(hand_outputs)
        spin(0.15)
        assert counts == (len(arm_outputs), len(hand_outputs))
        call(stop)
        grip = 1.0
        spin(1.3)
        call(takeover)
        spin(0.3)
        assert gate._phase == "idle" and counts == (
            len(arm_outputs),
            len(hand_outputs),
        )
        grip = 0.0
        trigger = max(0.0, min(1.0, 1.0 - feedback[-1].position[0] / 0.085))
        spin(0.2)
        grip = 1.0
        spin(1.6)
        assert gate._phase == "active", (gate._phase, gate._reason)
        live = False
        spin(0.5)
        assert gate._phase == "idle"
        counts = len(arm_outputs), len(hand_outputs)
        live = True
        spin(0.5)
        assert gate._phase == "idle" and counts == (
            len(arm_outputs),
            len(hand_outputs),
        )
        harness.get_logger().info(
            "PASS: matching, baseline cutover, old-command rejection, "
            "active mismatch, release, Manager STOP/TAKEOVER, held recovery"
        )


if __name__ == "__main__":
    main()
