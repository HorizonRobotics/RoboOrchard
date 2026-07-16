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

"""Stub the ROS/hardware modules when absent.

Lets the pure-python tests (calibration store loaders, oscillation
guard) run on a host without a ROS environment. Inside the container the
real modules import first and no stub is installed.
"""

import sys
import types


def _stub(name: str, attrs: tuple[str, ...] = ()) -> None:
    try:
        __import__(name)
        return
    except ImportError:
        pass
    module = types.ModuleType(name)
    for attr in attrs:
        setattr(module, attr, type(attr, (), {}))
    sys.modules[name] = module


_stub("geometry_msgs")
_stub("geometry_msgs.msg", ("PoseStamped",))
_stub("sensor_msgs")
_stub("sensor_msgs.msg", ("JointState",))
_stub("piper_sdk", ("C_PiperInterface",))
_stub("robo_orchard_piper_msg_ros2")
_stub("robo_orchard_piper_msg_ros2.msg", ("PiperStatusMsg",))
