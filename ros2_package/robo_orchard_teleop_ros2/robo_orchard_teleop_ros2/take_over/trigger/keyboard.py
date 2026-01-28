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


import json
import threading

import rclpy
from rclpy.node import Node
from sshkeyboard import listen_keyboard, stop_listening
from std_srvs.srv import Trigger


class Colors:
    """ANSI color codes for terminal output."""

    # Using bright colors for better visibility
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"
    # ANSI code to clear the line from the cursor to the end
    CLEAR_LINE = "\033[K"


class KeyboardTriggerNode(Node):
    """A single node that maps multiple string commands (e.g., 'trigger', 'release') to different sets of ROS2 services."""  # noqa: E501

    def __init__(self):
        super().__init__("multi_keyboard_trigger_node")

        # 1. Declare and get the key-to-service mapping parameter
        # This parameter is expected to be a YAML string or loaded from a file.
        self.declare_parameter("config", "{}")
        config = self.get_parameter("config").value

        try:
            with open(config, "r") as fh:
                self._key_service_map = json.load(fh)
            if not isinstance(self._key_service_map, dict):
                raise ValueError(
                    "Parameter 'key_service_map' must be a dictionary."
                )
        except Exception as e:
            self.get_logger().fatal(f"Failed to parse 'key_service_map': {e}")
            raise e

        # --- Internal State ---
        self._service_clients = {}
        self._input_buffer = ""
        self._lock = threading.Lock()

        # --- Setup from Parameters ---
        self.get_logger().info("Configuring multi-keyboard trigger...")

        # 2. Create clients for all services found in the map
        for key, service_list in self._key_service_map.items():
            self.get_logger().info(
                f"- Type `{key}` and press Enter ==> Calls {service_list}"
            )
            for service_name in service_list:
                if service_name not in self._service_clients:
                    self._service_clients[service_name] = self.create_client(
                        Trigger, service_name
                    )

        valid_keys = self._key_service_map.keys()
        if valid_keys:
            # Wrap the list of keys with color codes
            colored_keys_str = f"{Colors.BOLD}{Colors.CYAN}[ {', '.join(valid_keys)} ]{Colors.RESET}"  # noqa: E501
            hint_message = f"Hint: Valid commands are -> {colored_keys_str}"
            self.get_logger().info(hint_message)

        # --- Keyboard Listener in a separate thread ---
        self._listener_thread = threading.Thread(
            target=self._keyboard_listener_loop, daemon=True
        )
        self._listener_thread.start()
        self.get_logger().info(
            "Keyboard listener started. Ready for triggers."
        )

    def _keyboard_listener_loop(self):
        listen_keyboard(
            on_press=self._on_key_press, delay_second_char=0.05, lower=False
        )

    def _on_key_press(self, key: str):
        with self._lock:
            if key == "enter":
                # Check if the buffer matches any key in our map
                if self._input_buffer in self._key_service_map:
                    self._trigger_services_for_key(self._input_buffer)
                else:
                    self.get_logger().warn(
                        f"No action defined for input: '{self._input_buffer}'"
                    )

                # Reset buffer after Enter is pressed
                self._input_buffer = ""
            elif key == "backspace":
                self._input_buffer = self._input_buffer[:-1]
            elif len(key) == 1:
                self._input_buffer += key

            print(
                f"Input: {Colors.BOLD}{Colors.CYAN}"
                f"{self._input_buffer}"
                f"{Colors.RESET}{Colors.CLEAR_LINE}",
                end="\r",
                flush=True,
            )

    def _trigger_services_for_key(self, key: str):
        service_names = self._key_service_map.get(key, [])
        self.get_logger().info(
            f"Key binding `{key}` matched. Calling {len(service_names)} service(s)."  # noqa: E501
        )

        for service_name in service_names:
            client = self._service_clients.get(service_name)
            if not client or not client.service_is_ready():
                self.get_logger().warn(
                    f"Service `{service_name}` is not available. Trigger ignored."  # noqa: E501
                )
                continue

            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda f, s=service_name: self.service_response_callback(f, s)
            )

    def service_response_callback(self, future, service_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Service `{service_name}` successful: {response.message}"
                )
            else:
                self.get_logger().warn(
                    f"Service `{service_name}` failed: {response.message}"
                )
        except Exception as e:
            self.get_logger().error(
                f"Service `{service_name}` call failed: {e}"
            )

    def destroy_node(self):
        self.get_logger().info("Stopping keyboard listener thread...")
        stop_listening()
        if (
            hasattr(self, "_listener_thread")
            and self._listener_thread.is_alive()
        ):
            self._listener_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KeyboardTriggerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        if "node" in locals() and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
