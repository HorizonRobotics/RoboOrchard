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

from pathlib import Path

from robo_orchard_teleop_ros2.activation.linux_input import (
    InputDeviceIdentity,
    InputDeviceMatch,
    device_matches,
    find_matching_device,
)


def _write(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def _device_tree(tmp_path: Path, serial: str = "SERIAL-123"):
    input_root = tmp_path / "dev" / "input"
    sys_class_root = tmp_path / "sys" / "class" / "input"
    input_root.mkdir(parents=True)
    (input_root / "event22").touch()

    usb_interface = tmp_path / "devices" / "3-11.1:1.0"
    input_device = usb_interface / "hid" / "input" / "input38"
    _write(usb_interface / "bInterfaceNumber", "00\n")
    _write(input_device / "id" / "vendor", "8089\n")
    _write(input_device / "id" / "product", "000c\n")
    _write(input_device / "id" / "bustype", "0003\n")
    _write(input_device / "name", "USB Keyboard\n")
    _write(input_device / "uniq", f"{serial}\n")

    event_class = sys_class_root / "event22"
    event_class.mkdir(parents=True)
    (event_class / "device").symlink_to(input_device, target_is_directory=True)
    return input_root, sys_class_root


def test_find_matching_keyboard_by_stable_identity(tmp_path):
    input_root, sys_class_root = _device_tree(tmp_path)
    identity = find_matching_device(
        InputDeviceMatch(
            vendor_id="8089",
            product_id="000c",
            serial="SERIAL-123",
            interface_number=0,
        ),
        input_root=str(input_root),
        sys_class_input_root=str(sys_class_root),
    )

    assert identity is not None
    assert identity.name == "USB Keyboard"


def test_empty_serial_accepts_keyboard_without_serial_binding(tmp_path):
    input_root, sys_class_root = _device_tree(tmp_path, serial="")
    identity = find_matching_device(
        InputDeviceMatch(
            vendor_id="8089",
            product_id="000c",
            serial="",
            interface_number=0,
        ),
        input_root=str(input_root),
        sys_class_input_root=str(sys_class_root),
    )

    assert identity is not None


def test_device_name_disambiguates_event_nodes():
    identity = InputDeviceIdentity(
        event_path="/dev/input/event12",
        name="SayoDevice CM6K Keyboard",
        vendor_id="8089",
        product_id="000c",
        serial="",
        interface_number=1,
        bus_type="0003",
    )

    assert device_matches(
        identity,
        InputDeviceMatch(
            name="SayoDevice CM6K Keyboard",
            vendor_id="8089",
            product_id="000c",
            serial="",
            interface_number=1,
        ),
    )
    assert not device_matches(
        identity,
        InputDeviceMatch(
            name="SayoDevice CM6K",
            vendor_id="8089",
            product_id="000c",
            serial="",
            interface_number=1,
        ),
    )
