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

import errno
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty

from robo_orchard_teleop_msg_ros2.msg import TeleopActivationState
from robo_orchard_teleop_ros2.activation.linux_input import (
    EV_KEY,
    INPUT_EVENT_STRUCT,
    InputDeviceMatch,
    find_matching_device,
    open_input_device,
    read_key_pressed,
    unpack_input_events,
)
from robo_orchard_teleop_ros2.activation.state_machine import (
    ActivationState,
    HoldActionStateMachine,
    HoldActivationStateMachine,
)


class TeleopKeyboardNode(Node):
    """Publish teleop activation state and reset events from Linux keys."""

    def __init__(self) -> None:
        super().__init__("teleop_keyboard_node")
        self._declare_parameters()
        self._validate_parameters()

        self._activation_topic = str(
            self.get_parameter("activation_topic").value
        )
        self._reset_topic = str(self.get_parameter("reset_topic").value)
        self._activation_key_code = int(
            self.get_parameter("activation_key_code").value
        )
        self._activation_key_name = str(
            self.get_parameter("activation_key_name").value
        )
        self._reset_key_code = int(self.get_parameter("reset_key_code").value)
        self._reset_key_name = str(self.get_parameter("reset_key_name").value)
        self._input_root = str(self.get_parameter("input_root").value)
        self._sys_class_input_root = str(
            self.get_parameter("sys_class_input_root").value
        )
        self._device_match = InputDeviceMatch(
            name=str(self.get_parameter("device.name").value),
            vendor_id=str(self.get_parameter("device.vendor_id").value),
            product_id=str(self.get_parameter("device.product_id").value),
            serial=str(self.get_parameter("device.serial").value),
            interface_number=int(
                self.get_parameter("device.interface_number").value
            ),
            bus_type=str(self.get_parameter("device.bus_type").value),
        )

        activation_hold_s = (
            float(self.get_parameter("activation_hold_ms").value) / 1000.0
        )
        reset_hold_s = (
            float(self.get_parameter("reset_hold_ms").value) / 1000.0
        )
        self._activation = HoldActivationStateMachine(activation_hold_s)
        self._reset = HoldActionStateMachine(reset_hold_s)
        self._file_descriptor: int | None = None
        self._connected_path = ""
        self._read_buffer = bytearray()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._activation_publisher = self.create_publisher(
            TeleopActivationState, self._activation_topic, qos
        )
        self._reset_publisher = self.create_publisher(
            Empty, self._reset_topic, qos
        )

        poll_interval_s = (
            float(self.get_parameter("poll_interval_ms").value) / 1000.0
        )
        reconnect_interval_s = float(
            self.get_parameter("reconnect_interval_s").value
        )
        heartbeat_hz = float(self.get_parameter("heartbeat_hz").value)
        self._poll_timer = self.create_timer(poll_interval_s, self._poll)
        self._reconnect_timer = self.create_timer(
            reconnect_interval_s, self._try_connect
        )
        self._heartbeat_timer = self.create_timer(
            1.0 / heartbeat_hz, self._publish_activation
        )

        self._try_connect()
        self._publish_activation()

    def _declare_parameters(self) -> None:
        self.declare_parameter("activation_topic", "/teleop/activation/state")
        self.declare_parameter("reset_topic", "/teleop/reset")
        self.declare_parameter("device.bus_type", "0003")
        self.declare_parameter("device.name", "")
        self.declare_parameter("device.vendor_id", "")
        self.declare_parameter("device.product_id", "")
        self.declare_parameter("device.serial", "")
        self.declare_parameter("device.interface_number", -1)
        self.declare_parameter("activation_key_code", 20)
        self.declare_parameter("activation_key_name", "KEY_T")
        self.declare_parameter("reset_key_code", 19)
        self.declare_parameter("reset_key_name", "KEY_R")
        self.declare_parameter("activation_hold_ms", 1000.0)
        self.declare_parameter("reset_hold_ms", 1000.0)
        self.declare_parameter("poll_interval_ms", 5.0)
        self.declare_parameter("heartbeat_hz", 30.0)
        self.declare_parameter("reconnect_interval_s", 1.0)
        self.declare_parameter("input_root", "/dev/input")
        self.declare_parameter("sys_class_input_root", "/sys/class/input")

    def _validate_parameters(self) -> None:
        for name in (
            "activation_topic",
            "reset_topic",
            "device.vendor_id",
            "device.product_id",
        ):
            if not str(self.get_parameter(name).value).strip():
                raise ValueError(f"Parameter '{name}' must not be empty")
        for name in (
            "device.interface_number",
            "activation_key_code",
            "reset_key_code",
        ):
            if int(self.get_parameter(name).value) < 0:
                raise ValueError(f"Parameter '{name}' must be non-negative")
        for name in (
            "poll_interval_ms",
            "heartbeat_hz",
            "reconnect_interval_s",
            # A zero activation hold lets a release and the following press
            # land in one poll batch, which publishes ACTIVE without ever
            # publishing the INACTIVE between them. Consumers re-arm only on
            # an observed INACTIVE, so they would stay locked out.
            "activation_hold_ms",
        ):
            if float(self.get_parameter(name).value) <= 0.0:
                raise ValueError(f"Parameter '{name}' must be positive")
        if float(self.get_parameter("reset_hold_ms").value) < 0.0:
            raise ValueError("Parameter 'reset_hold_ms' must be non-negative")
        if (
            self.get_parameter("activation_key_code").value
            == self.get_parameter("reset_key_code").value
        ):
            raise ValueError("Activation and reset keys must be different")

    def _try_connect(self) -> None:
        if self._file_descriptor is not None:
            return
        identity = find_matching_device(
            self._device_match,
            input_root=self._input_root,
            sys_class_input_root=self._sys_class_input_root,
        )
        if identity is None:
            return

        try:
            file_descriptor = open_input_device(identity.event_path)
            activation_pressed = read_key_pressed(
                file_descriptor, self._activation_key_code
            )
            reset_pressed = read_key_pressed(
                file_descriptor, self._reset_key_code
            )
        except OSError as error:
            if "file_descriptor" in locals():
                os.close(file_descriptor)
            self.get_logger().warning(
                f"Cannot open input device {identity.event_path}: {error}"
            )
            return

        self._file_descriptor = file_descriptor
        self._connected_path = identity.event_path
        self._read_buffer.clear()
        self._activation.connect(activation_pressed)
        self._reset.connect(reset_pressed)
        self.get_logger().info(
            f"Connected {identity.name} at {identity.event_path}; "
            f"activation={self._activation_key_name} "
            f"(code {self._activation_key_code}), "
            f"reset={self._reset_key_name} (code {self._reset_key_code})"
        )
        self._publish_activation()

    def _poll(self) -> None:
        if self._file_descriptor is None:
            return
        try:
            while True:
                data = os.read(
                    self._file_descriptor, INPUT_EVENT_STRUCT.size * 64
                )
                if not data:
                    self._disconnect("input device returned EOF")
                    return
                self._read_buffer.extend(data)
        except BlockingIOError:
            pass
        except OSError as error:
            if error.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self._disconnect(str(error))
                return

        now = time.monotonic()
        activation_changed = False
        for event_type, code, value in unpack_input_events(self._read_buffer):
            if event_type != EV_KEY:
                continue
            if code == self._activation_key_code:
                activation_changed |= self._activation.key_event(value, now)
            elif code == self._reset_key_code:
                self._reset.key_event(value, now)
        activation_changed |= self._activation.advance(now)
        if self._reset.advance(now):
            self._reset_publisher.publish(Empty())
            self.get_logger().info("Keyboard reset requested")
        if activation_changed:
            state_name = ActivationState(self._activation.state).name
            self.get_logger().info(
                f"Keyboard activation changed to {state_name} "
                f"(transition {self._activation.transition_id})"
            )
            self._publish_activation()

    def _disconnect(self, reason: str) -> None:
        path = self._connected_path
        if self._file_descriptor is not None:
            try:
                os.close(self._file_descriptor)
            except OSError:
                pass
        self._file_descriptor = None
        self._connected_path = ""
        self._read_buffer.clear()
        changed = self._activation.disconnect()
        self._reset.disconnect()
        self.get_logger().error(
            f"Lost input device {path}: {reason}; keyboard input unavailable"
        )
        if changed:
            self._publish_activation()

    def _publish_activation(self) -> None:
        message = TeleopActivationState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = int(self._activation.state)
        message.transition_id = self._activation.transition_id
        self._activation_publisher.publish(message)

    def destroy_node(self) -> bool:
        if self._file_descriptor is not None:
            os.close(self._file_descriptor)
            self._file_descriptor = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopKeyboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
