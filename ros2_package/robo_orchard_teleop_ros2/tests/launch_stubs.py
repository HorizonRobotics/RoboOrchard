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

from __future__ import annotations
import importlib.util
import sys
import types
from pathlib import Path


class LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=""):
        self.name = name
        self.default_value = default_value
        self.description = description


class IncludeLaunchDescription:
    def __init__(self, source, launch_arguments=None):
        self.source = source
        self.launch_arguments = list(launch_arguments or [])


class PythonLaunchDescriptionSource:
    def __init__(self, location):
        self.location = location


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class OpaqueFunction:
    def __init__(self, function):
        self.function = function


class PathJoinSubstitution:
    def __init__(self, substitutions):
        self.substitutions = list(substitutions)


class FindPackageShare:
    def __init__(self, package):
        self.package = package


class Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def load_launch_module(path: Path, module_name: str):
    """Load a Python launch file without requiring a ROS installation."""
    modules = {
        "launch": types.ModuleType("launch"),
        "launch.actions": types.ModuleType("launch.actions"),
        "launch.launch_description_sources": types.ModuleType(
            "launch.launch_description_sources"
        ),
        "launch.substitutions": types.ModuleType("launch.substitutions"),
        "launch_ros": types.ModuleType("launch_ros"),
        "launch_ros.actions": types.ModuleType("launch_ros.actions"),
        "launch_ros.substitutions": types.ModuleType(
            "launch_ros.substitutions"
        ),
    }
    modules["launch"].LaunchDescription = LaunchDescription
    modules["launch.actions"].DeclareLaunchArgument = DeclareLaunchArgument
    modules[
        "launch.actions"
    ].IncludeLaunchDescription = IncludeLaunchDescription
    modules["launch.actions"].OpaqueFunction = OpaqueFunction
    modules[
        "launch.launch_description_sources"
    ].PythonLaunchDescriptionSource = PythonLaunchDescriptionSource
    modules["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
    modules["launch.substitutions"].PathJoinSubstitution = PathJoinSubstitution
    modules["launch_ros.actions"].Node = Node
    modules["launch_ros.substitutions"].FindPackageShare = FindPackageShare

    helper_name = "robo_orchard_teleop_ros2.wuji_teleop_launch"
    saved = {name: sys.modules.get(name) for name in (*modules, helper_name)}
    sys.modules.update(modules)
    sys.modules.pop(helper_name, None)
    try:
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
