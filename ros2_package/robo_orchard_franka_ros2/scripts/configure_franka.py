#!/usr/bin/env python3
# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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

"""Apply official robot settings once before the position controller starts."""

import math
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import rclpy
import yaml
from franka_msgs.srv import (
    SetCartesianStiffness,
    SetForceTorqueCollisionBehavior,
    SetFullCollisionBehavior,
    SetJointStiffness,
    SetLoad,
    SetStiffnessFrame,
    SetTCPFrame,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


def _number(value: object, field: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number (not a boolean)")
    return float(value)


def _array(value: object, size: int, field: str) -> list[float]:
    if (
        isinstance(value, (str, bytes))
        or not isinstance(value, Sequence)
        or len(value) != size
    ):
        raise ValueError(f"{field} must contain {size} numbers")
    return [_number(item, field) for item in value]


def _build_requests(config: dict[str, object]) -> list[tuple[str, type, Any]]:
    """Validate all settings and construct typed requests before any writes."""
    unknown = config.keys() - {
        "joint_stiffness",
        "cartesian_stiffness",
        "load",
        "tcp_frame",
        "stiffness_frame",
        "force_torque_collision_behavior",
        "full_collision_behavior",
    }
    if unknown:
        raise ValueError(
            f"Unknown robot configuration sections: {list(unknown)}"
        )
    if (
        "force_torque_collision_behavior" in config
        and "full_collision_behavior" in config
    ):
        raise ValueError("Specify only one collision behavior section")
    requests = []
    for section, values in config.items():
        if section == "joint_stiffness":
            service_type = SetJointStiffness
            fields = {"joint_stiffness": 7}
        elif section == "cartesian_stiffness":
            service_type = SetCartesianStiffness
            fields = {"cartesian_stiffness": 6}
        elif section == "load":
            service_type = SetLoad
            fields = {"mass": None, "center_of_mass": 3, "load_inertia": 9}
        elif section == "tcp_frame":
            service_type = SetTCPFrame
            fields = {"transformation": 16}
        elif section == "stiffness_frame":
            service_type = SetStiffnessFrame
            fields = {"transformation": 16}
        else:
            service_type = (
                SetFullCollisionBehavior
                if section == "full_collision_behavior"
                else SetForceTorqueCollisionBehavior
            )
            phases = (
                ("nominal", "acceleration")
                if section == "full_collision_behavior"
                else ("nominal",)
            )
            fields = {
                f"{bound}_{kind}_thresholds_{phase}": size
                for phase in phases
                for kind, size in (("torque", 7), ("force", 6))
                for bound in ("lower", "upper")
            }
        if not isinstance(values, dict) or values.keys() != fields.keys():
            raise ValueError(
                f"{section} requires exactly these fields: {list(fields)}"
            )
        request = service_type.Request()
        for field, size in fields.items():
            name = f"{section}.{field}"
            value = (
                _number(values[field], name)
                if size is None
                else _array(values[field], size, name)
            )
            if field == "mass" and value < 0.0:
                raise ValueError("load.mass must be nonnegative (kg)")
            if field == "joint_stiffness" and any(
                not 0.0 <= gain <= 14250.0 for gain in value
            ):
                raise ValueError(
                    "Joint stiffness must be in [0, 14250] Nm/rad"
                )
            if field == "cartesian_stiffness" and any(
                not (
                    10.0 <= gain <= 3000.0
                    if index < 3
                    else 1.0 <= gain <= 300.0
                )
                for index, gain in enumerate(value)
            ):
                raise ValueError(
                    "Cartesian stiffness must be in [10, 3000] N/m for xyz "
                    "and [1, 300] Nm/rad for roll/pitch/yaw"
                )
            if "thresholds" in field and any(item < 0.0 for item in value):
                raise ValueError(f"{name} must be nonnegative")
            setattr(request, field, value)
        for field in fields:
            if field.startswith("lower_") and any(
                lower > upper
                for lower, upper in zip(
                    getattr(request, field),
                    getattr(request, field.replace("lower_", "upper_", 1)),
                    strict=True,
                )
            ):
                raise ValueError(
                    f"{section}: upper thresholds must be >= lower"
                )
        requests.append((section, service_type, request))
    return requests


def configure_robot(node: Node) -> None:
    """Apply launch stiffness and optional YAML settings before activation.

    YAML joint stiffness overrides the launch gains. All other omitted sections
    remain untouched. Validate and build every request before applying joint
    stiffness first, then optional sections in file order. Each service has
    separate 30-second discovery and response bounds. A failure stops further
    writes without retry, recovery or rollback of already accepted settings.
    """
    filename = node.declare_parameter("robot_config_file", "").value
    config = {}
    if filename:
        with Path(filename).open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError("robot_config_file must contain a YAML mapping")
    if "joint_stiffness" in config:
        stiffness = config.pop("joint_stiffness")
    else:
        gains = node.declare_parameter(
            "joint_stiffness",
            "",
            descriptor=ParameterDescriptor(dynamic_typing=True),
        ).value
        # Launch passes the list as text; direct ROS callers may pass an array.
        if isinstance(gains, str):
            gains = yaml.safe_load(gains)
        stiffness = {"joint_stiffness": gains}
    requests = _build_requests({"joint_stiffness": stiffness, **config})
    for section, service_type, request in requests:
        service_name = f"service_server/set_{section}"
        client = node.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"{service_name} unavailable after 30 s")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done():
            raise RuntimeError(f"{service_name} response timed out after 30 s")
        response = future.result()
        if response is None or not response.success:
            error = response.error if response is not None else "No response"
            raise RuntimeError(f"{service_name} failed: {error}")
        if section == "joint_stiffness":
            node.get_logger().info(
                "Applied joint stiffness (Nm/rad): "
                f"{list(request.joint_stiffness)}"
            )
        else:
            node.get_logger().info(
                f"Applied robot configuration section: {section}"
            )


def main(args: list[str] | None = None) -> int:
    """Exit successfully only after every requested setting is acknowledged."""
    rclpy.init(args=args)
    node = Node("configure_franka")
    try:
        configure_robot(node)
        return 0
    except Exception as error:
        node.get_logger().error(f"Cannot configure robot: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
