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
# ruff: noqa: E402
import importlib
import sys
import types
from dataclasses import dataclass
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

_saved_numpy_module = sys.modules.pop("numpy", None)
import numpy as np

if _saved_numpy_module is not None:
    sys.modules["numpy"] = _saved_numpy_module

_STUB_MODULE_NAMES = (
    "geometry_msgs",
    "geometry_msgs.msg",
    "ikpy",
    "ikpy.chain",
    "numpy",
    "scipy",
    "scipy.spatial",
    "scipy.spatial.transform",
)


@dataclass
class _Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class _Pose:
    position: _Point
    orientation: _Quaternion


class _Rotation:
    def __init__(self, matrix=None, magnitude_value=0.0):
        self._matrix = (
            np.array(matrix, dtype=np.float64)
            if matrix is not None
            else np.eye(3, dtype=np.float64)
        )
        self._magnitude_value = float(magnitude_value)

    @classmethod
    def from_quat(cls, quat):
        quat = list(quat)
        assert quat == [0.0, 0.0, 0.0, 1.0]
        return cls(np.eye(3, dtype=np.float64), 0.0)

    @classmethod
    def from_matrix(cls, matrix):
        matrix = np.array(matrix, dtype=np.float64)
        magnitude_value = (
            0.0 if np.allclose(matrix, np.eye(3, dtype=np.float64)) else 1.0
        )
        return cls(matrix, magnitude_value)

    def as_matrix(self):
        return np.array(self._matrix, dtype=np.float64)

    def magnitude(self):
        return self._magnitude_value


class _Link:
    def __init__(self, name, joint_type="revolute", bounds=None):
        self.name = name
        self.joint_type = joint_type
        self.bounds = bounds


class _Chain:
    next_result = None
    next_fk_matrix = np.eye(4, dtype=np.float64)
    last_init_kwargs = None
    last_inverse_target = None
    last_inverse_kwargs = None
    last_forward_input = None

    def __init__(self):
        self.links = [
            _Link("base_link", joint_type="fixed"),
            _Link("joint1", bounds=(-1.0, 1.0)),
            _Link("joint2", bounds=(-2.0, 2.0)),
            _Link("joint3", bounds=(-3.0, 3.0)),
            _Link("joint4", bounds=(-4.0, 4.0)),
            _Link("joint5", bounds=(-5.0, 5.0)),
            _Link("joint6", bounds=(-6.0, 6.0)),
        ]
        self.active_links_mask = None

    @classmethod
    def from_urdf_file(cls, *args, **kwargs):
        cls.last_init_kwargs = {"args": args, "kwargs": kwargs}
        return cls()

    def inverse_kinematics_frame(self, target_matrix, **kwargs):
        type(self).last_inverse_target = np.array(
            target_matrix, dtype=np.float64
        )
        type(self).last_inverse_kwargs = dict(kwargs)
        result = type(self).next_result
        if isinstance(result, Exception):
            raise result
        return np.array(result, dtype=np.float64)

    def forward_kinematics(self, result):
        type(self).last_forward_input = np.array(result, dtype=np.float64)
        return np.array(type(self).next_fk_matrix, dtype=np.float64)


def _install_stub_modules():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Pose = _Pose
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    numpy_module = types.ModuleType("numpy")
    numpy_module.array = np.array
    numpy_module.eye = np.eye
    numpy_module.linalg = types.SimpleNamespace(norm=np.linalg.norm)
    sys.modules["numpy"] = numpy_module

    scipy_module = types.ModuleType("scipy")
    scipy_spatial_module = types.ModuleType("scipy.spatial")
    scipy_transform_module = types.ModuleType("scipy.spatial.transform")
    scipy_transform_module.Rotation = _Rotation
    scipy_spatial_module.transform = scipy_transform_module
    scipy_module.spatial = scipy_spatial_module
    sys.modules["scipy"] = scipy_module
    sys.modules["scipy.spatial"] = scipy_spatial_module
    sys.modules["scipy.spatial.transform"] = scipy_transform_module

    ikpy_module = types.ModuleType("ikpy")
    ikpy_chain_module = types.ModuleType("ikpy.chain")
    ikpy_chain_module.Chain = _Chain
    ikpy_module.chain = ikpy_chain_module
    sys.modules["ikpy"] = ikpy_module
    sys.modules["ikpy.chain"] = ikpy_chain_module


@pytest.fixture()
def ik_module(tmp_path):
    module_name = "robo_orchard_teleop_ros2.ik"
    saved_modules = {
        name: sys.modules.get(name)
        for name in (*_STUB_MODULE_NAMES, module_name)
    }

    for name in (*_STUB_MODULE_NAMES, module_name):
        sys.modules.pop(name, None)

    _install_stub_modules()
    module = importlib.import_module(module_name)

    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text("<robot/>", encoding="utf-8")
    module._TEST_URDF_PATH = str(urdf_path)

    _Chain.next_result = None
    _Chain.next_fk_matrix = np.eye(4, dtype=np.float64)
    _Chain.last_init_kwargs = None
    _Chain.last_inverse_target = None
    _Chain.last_inverse_kwargs = None
    _Chain.last_forward_input = None

    try:
        yield module
    finally:
        sys.modules.pop(module_name, None)
        for name in _STUB_MODULE_NAMES:
            sys.modules.pop(name, None)

        for name, saved_module in saved_modules.items():
            if saved_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = saved_module


def test_ik_optimizer_builds_chain_and_uses_seeded_initial_position(ik_module):
    _Chain.next_result = [0.0, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    fk = np.eye(4, dtype=np.float64)
    fk[:3, 3] = [0.1, 0.2, 0.3]
    _Chain.next_fk_matrix = fk
    optimizer = ik_module.IkOptimizer(
        urdf_path=ik_module._TEST_URDF_PATH,
        base_link="base_link",
        ee_link="link6",
    )
    pose = _Pose(
        position=_Point(0.1, 0.2, 0.3),
        orientation=_Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    solution = optimizer.solve(
        pose, seed_state=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    )

    assert solution == [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    assert _Chain.last_init_kwargs is not None
    assert _Chain.last_init_kwargs["kwargs"]["name"] == "link6"
    assert _Chain.last_inverse_kwargs["orientation_mode"] == "all"
    assert _Chain.last_inverse_kwargs["regularization_parameter"] == 0.02
    assert _Chain.last_inverse_kwargs["initial_position"] == [
        0.0,
        0.1,
        -0.2,
        0.3,
        -0.4,
        0.5,
        -0.6,
    ]


def test_ik_optimizer_clips_seed_and_solution_to_joint_limits(ik_module):
    _Chain.next_result = [0.0, 2.0, -3.0, 4.0, -5.0, 6.0, -7.0]
    _Chain.next_fk_matrix = np.eye(4, dtype=np.float64)
    optimizer = ik_module.IkOptimizer(
        urdf_path=ik_module._TEST_URDF_PATH,
        base_link="base_link",
        ee_link="link6",
    )
    pose = _Pose(
        position=_Point(0.0, 0.0, 0.0),
        orientation=_Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    solution = optimizer.solve(
        pose, seed_state=[2.0, -3.0, 4.0, -5.0, 6.0, -7.0]
    )

    assert _Chain.last_inverse_kwargs["initial_position"] == [
        0.0,
        1.0,
        -2.0,
        3.0,
        -4.0,
        5.0,
        -6.0,
    ]
    assert _Chain.last_inverse_kwargs["regularization_parameter"] == 0.02
    assert solution == [1.0, -2.0, 3.0, -4.0, 5.0, -6.0]


def test_ik_optimizer_returns_none_on_solver_exception(ik_module):
    _Chain.next_result = RuntimeError("boom")
    optimizer = ik_module.IkOptimizer(
        urdf_path=ik_module._TEST_URDF_PATH,
        base_link="base_link",
        ee_link="link6",
    )
    pose = _Pose(
        position=_Point(0.0, 0.0, 0.0),
        orientation=_Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    assert optimizer.solve(pose, seed_state=[0.0] * 6) is None


def test_ik_optimizer_returns_none_on_large_fk_error(ik_module):
    _Chain.next_result = [0.0] * 7
    fk = np.eye(4, dtype=np.float64)
    fk[0, 3] = 0.1
    _Chain.next_fk_matrix = fk
    optimizer = ik_module.IkOptimizer(
        urdf_path=ik_module._TEST_URDF_PATH,
        base_link="base_link",
        ee_link="link6",
    )
    pose = _Pose(
        position=_Point(0.0, 0.0, 0.0),
        orientation=_Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    assert optimizer.solve(pose, seed_state=[0.0] * 6) is None


def test_ik_optimizer_validates_seed_length(ik_module):
    optimizer = ik_module.IkOptimizer(
        urdf_path=ik_module._TEST_URDF_PATH,
        base_link="base_link",
        ee_link="link6",
    )
    pose = _Pose(
        position=_Point(0.0, 0.0, 0.0),
        orientation=_Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    with pytest.raises(ValueError, match="seed_state length 2"):
        optimizer.solve(pose, seed_state=[0.0, 0.0])


def test_ik_optimizer_skips_regularization_without_seed(ik_module):
    _Chain.next_result = [0.0] * 7
    _Chain.next_fk_matrix = np.eye(4, dtype=np.float64)
    optimizer = ik_module.IkOptimizer(
        urdf_path=ik_module._TEST_URDF_PATH,
        base_link="base_link",
        ee_link="link6",
    )
    pose = _Pose(
        position=_Point(0.0, 0.0, 0.0),
        orientation=_Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    solution = optimizer.solve(pose, seed_state=None)

    assert solution == [0.0] * 6
    assert "regularization_parameter" not in _Chain.last_inverse_kwargs
