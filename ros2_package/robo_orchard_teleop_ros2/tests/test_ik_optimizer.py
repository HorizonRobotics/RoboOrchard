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

from __future__ import annotations
import importlib
import sys
from types import SimpleNamespace

import pytest


class _Ik:
    result = None
    calls = []

    def __init__(self, *args):
        self.joint_names = ["joint1", "joint2"]
        self.num_joints = 2
        type(self).calls.append(args)

    def solve_pose(self, pose, **kwargs):
        type(self).calls.append((pose, kwargs))
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


@pytest.fixture()
def ik_module(monkeypatch):
    extension_name = "robo_orchard_teleop_ros2._ik"
    module_name = "robo_orchard_teleop_ros2.ik"
    monkeypatch.setitem(sys.modules, extension_name, SimpleNamespace(Ik=_Ik))
    monkeypatch.setitem(
        sys.modules, "geometry_msgs.msg", SimpleNamespace(Pose=object)
    )
    monkeypatch.delitem(sys.modules, module_name, raising=False)
    _Ik.calls = []
    _Ik.result = {
        "q": [0.1, -0.2],
        "accepted": True,
        "position_error": 0.0,
        "rotation_error": 0.0,
    }
    module = importlib.import_module(module_name)
    yield module
    sys.modules.pop(module_name, None)


def pose(quat=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(**dict(zip("xyzw", quat, strict=True))),
    )


def optimizer(module, **kwargs):
    return module.IkOptimizer("robot.urdf", "base", "ee", **kwargs)


def test_solver_chain_and_seeded_options(ik_module):
    ik = optimizer(ik_module)
    assert ik.get_joint_names() == ["joint1", "joint2"]
    assert ik.num_joints == 2
    assert _Ik.calls[0] == ("robot.urdf", "base", "ee")
    assert ik.solve(pose(), [0.4, -0.5]) == [0.1, -0.2]
    values, kwargs = _Ik.calls[-1]
    assert values == [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
    assert kwargs == {
        "seed": [0.4, -0.5],
        "orientation_mode": "all",
        "regularization_parameter": 0.02,
        "clip_to_limit": True,
        "position_tolerance": 0.05,
        "orientation_tolerance": 0.3,
    }


@pytest.mark.parametrize("regularization", [None, 0.0, 0.07])
@pytest.mark.parametrize("mode", [None, "X", "Y", "Z", "all"])
def test_custom_options_are_forwarded(ik_module, regularization, mode):
    ik = optimizer(
        ik_module,
        regularization_parameter=regularization,
        clip_to_limit=False,
        position_tolerance=0.03,
        orientation_tolerance=0.2,
    )
    assert ik.solve(pose(), orientation_mode=mode) == [0.1, -0.2]
    _, kwargs = _Ik.calls[-1]
    assert kwargs["seed"] is None
    assert kwargs["regularization_parameter"] == regularization
    assert kwargs["orientation_mode"] == mode
    assert not kwargs["clip_to_limit"]
    assert kwargs["position_tolerance"] == 0.03
    assert kwargs["orientation_tolerance"] == 0.2


def test_solver_failures_keep_logging_and_return_none(ik_module):
    messages = []
    logger = SimpleNamespace(warning=messages.append)
    ik = optimizer(ik_module, logger=logger)
    _Ik.result = RuntimeError("boom")
    assert ik.solve(pose(), [0.0, 0.0]) is None
    assert messages == ["IK solver exception: boom"]
    _Ik.result = {
        "accepted": False,
        "position_error": 0.1,
        "rotation_error": 0.4,
    }
    assert ik.solve(pose()) is None
    assert messages[-1] == (
        "IK failed to converge (pos_err=0.1000, rot_err=0.4000)"
    )


def test_input_errors_remain_caller_errors(ik_module):
    ik = optimizer(ik_module)
    with pytest.raises(ValueError, match="seed_state length 1"):
        ik.solve(pose(), [0.0])
    for quat in [(0.0,) * 4, (float("nan"), 0.0, 0.0, 1.0)]:
        with pytest.raises(ValueError, match="zero norm"):
            ik.solve(pose(quat))
