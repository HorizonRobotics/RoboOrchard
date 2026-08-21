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

import fcntl
import glob
import os
import struct
from dataclasses import dataclass
from pathlib import Path

EV_KEY = 0x01
INPUT_EVENT_STRUCT = struct.Struct("@llHHI")

_IOC_READ = 2
_IOC_DIRSHIFT = 30
_IOC_SIZESHIFT = 16
_IOC_TYPESHIFT = 8
_EVIOCGKEY_NR = 0x18


@dataclass(frozen=True)
class InputDeviceMatch:
    """Stable identity used to find a Linux input event device."""

    vendor_id: str
    product_id: str
    serial: str
    interface_number: int
    bus_type: str = "0003"
    name: str = ""


@dataclass(frozen=True)
class InputDeviceIdentity:
    """Identity read from sysfs for one event device."""

    event_path: str
    name: str
    vendor_id: str
    product_id: str
    serial: str
    interface_number: int | None
    bus_type: str


def normalize_hex_id(value: str) -> str:
    """Normalize sysfs and configuration hexadecimal IDs for comparison."""
    stripped = value.strip().lower()
    if stripped.startswith("0x"):
        stripped = stripped[2:]
    return stripped.zfill(4)


def find_matching_device(
    expected: InputDeviceMatch,
    input_root: str = "/dev/input",
    sys_class_input_root: str = "/sys/class/input",
) -> InputDeviceIdentity | None:
    """Find the first event device whose stable hardware identity matches."""
    pattern = os.path.join(input_root, "event*")
    for event_path in sorted(glob.glob(pattern), key=_event_sort_key):
        event_name = os.path.basename(event_path)
        sys_device = Path(sys_class_input_root) / event_name / "device"
        identity = read_device_identity(event_path, sys_device)
        if identity is not None and device_matches(identity, expected):
            return identity
    return None


def read_device_identity(
    event_path: str, sys_device: Path
) -> InputDeviceIdentity | None:
    """Read one identity, returning None for incomplete sysfs data."""
    try:
        resolved_device = sys_device.resolve(strict=True)
        vendor_id = _read_text(sys_device / "id" / "vendor")
        product_id = _read_text(sys_device / "id" / "product")
        bus_type = _read_text(sys_device / "id" / "bustype")
        name = _read_text(sys_device / "name")
        serial = _read_optional_text(sys_device / "uniq")
        if not serial:
            serial = _find_parent_attribute(resolved_device, "serial") or ""
        interface_text = _find_parent_attribute(
            resolved_device,
            "bInterfaceNumber",
        )
        interface_number = (
            int(interface_text, 16) if interface_text is not None else None
        )
    except (OSError, ValueError):
        return None

    return InputDeviceIdentity(
        event_path=event_path,
        name=name,
        vendor_id=normalize_hex_id(vendor_id),
        product_id=normalize_hex_id(product_id),
        serial=serial,
        interface_number=interface_number,
        bus_type=normalize_hex_id(bus_type),
    )


def device_matches(
    actual: InputDeviceIdentity, expected: InputDeviceMatch
) -> bool:
    """Compare an input device against configured stable identifiers."""
    return (
        actual.vendor_id == normalize_hex_id(expected.vendor_id)
        and actual.product_id == normalize_hex_id(expected.product_id)
        and (not expected.name or actual.name == expected.name)
        and (not expected.serial or actual.serial == expected.serial)
        and actual.interface_number == expected.interface_number
        and actual.bus_type == normalize_hex_id(expected.bus_type)
    )


def open_input_device(path: str) -> int:
    """Open an evdev event file without taking an exclusive grab."""
    return os.open(path, os.O_RDONLY | os.O_NONBLOCK)


def read_key_pressed(file_descriptor: int, key_code: int) -> bool:
    """Query whether a key is already held when the event device is opened."""
    if key_code < 0:
        raise ValueError("key_code must be non-negative")

    byte_count = max(1, key_code // 8 + 1)
    key_bits = bytearray(byte_count)
    request = (
        (_IOC_READ << _IOC_DIRSHIFT)
        | (byte_count << _IOC_SIZESHIFT)
        | (ord("E") << _IOC_TYPESHIFT)
        | _EVIOCGKEY_NR
    )
    fcntl.ioctl(file_descriptor, request, key_bits, True)
    return bool(key_bits[key_code // 8] & (1 << (key_code % 8)))


def unpack_input_events(buffer: bytearray) -> list[tuple[int, int, int]]:
    """Remove and decode all complete input_event records from a buffer."""
    events = []
    offset = 0
    while len(buffer) - offset >= INPUT_EVENT_STRUCT.size:
        _, _, event_type, code, value = INPUT_EVENT_STRUCT.unpack_from(
            buffer, offset
        )
        events.append((event_type, code, value))
        offset += INPUT_EVENT_STRUCT.size
    if offset:
        del buffer[:offset]
    return events


def _event_sort_key(path: str) -> tuple[int, str]:
    name = os.path.basename(path)
    suffix = name.removeprefix("event")
    return (int(suffix), name) if suffix.isdigit() else (2**31, name)


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def _read_optional_text(path: Path) -> str:
    try:
        return _read_text(path)
    except OSError:
        return ""


def _find_parent_attribute(start: Path, attribute: str) -> str | None:
    for directory in (start, *start.parents):
        candidate = directory / attribute
        if candidate.is_file():
            return _read_text(candidate)
    return None
