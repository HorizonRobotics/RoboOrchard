# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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


import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from robo_orchard_pico_msg_ros2.msg import VRState
from robo_orchard_teleop_ros2.bridge.pico.intent import LongPressIntent


class PicoVRTriggerNode(Node):
    def __init__(self):
        super().__init__("pico_vr_trigger_node")

        self.declare_parameter("key", "trigger")
        self.declare_parameter("action_services", [""])
        self.declare_parameter("min_press_time", 0.5)

        key_binding_config = self.get_parameter("key").value
        action_services_config = self.get_parameter("action_services").value
        min_press_time = self.get_parameter("min_press_time").value

        if not key_binding_config:
            raise ValueError("Required key_binding_config")

        if not action_services_config:
            raise ValueError("Required action_services_config")

        # --- Internal State ---
        self._service_client = {}

        # 1. Create clients for all specified services
        for service_name in action_services_config:
            if not service_name:
                raise ValueError(f"Invalid service name: {service_name}")
            if service_name not in self._service_client:
                self._service_client[service_name] = self.create_client(
                    Trigger, service_name
                )

        self.intent_cls = LongPressIntent(
            key_binding_config,
            source_type="all",
            value_thresh=0.5,
            thresh=min_press_time,
        )

        self.msg_sub = self.create_subscription(
            VRState, "/pico_bridge/vr_state", self.vr_state_callback, 1
        )
        self._is_active = False

    def vr_state_callback(self, msg: VRState):
        if self.intent_cls.is_active(msg):
            if self._is_active:
                return
            self.get_logger().info(
                f"Hotkey `{self.intent_cls.button}` activated."
            )

            for service_name, client in self._service_client.items():
                if not client:
                    continue

                if not client.service_is_ready():
                    self.get_logger().warn(
                        f"Service `{service_name}` is not available. Trigger ignored."  # noqa: E501
                    )
                    continue

                future = client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f, s=service_name: self.service_response_callback(
                        f, s
                    )
                )

            self._is_active = True
        else:
            self._is_active = False

    def service_response_callback(self, future, service_name):
        """Callback to log the result of a single service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Service `{service_name}` call successful: {response.message}"  # noqa: E501
                )
            else:
                self.get_logger().warn(
                    f"Service `{service_name}` reported failure: {response.message}"  # noqa: E501
                )
        except Exception as e:
            self.get_logger().error(
                f"Service `{service_name}` call failed: {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PicoVRTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
