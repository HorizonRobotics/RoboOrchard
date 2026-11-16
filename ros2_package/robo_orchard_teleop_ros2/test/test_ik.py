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

"""Real IK extension tests; first build/install the ROS package."""

from __future__ import annotations
import importlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.optimize._numdiff import approx_derivative
from scipy.spatial.transform import Rotation

# This target runs after installation. Missing binaries, wrong install paths
# and unresolved shared libraries are test failures, not optional coverage.
ik_module = importlib.import_module("robo_orchard_teleop_ros2._ik")

ROOT = Path(__file__).resolve().parents[3]
MARVIN = ROOT / (
    "ros2_package/robo_orchard_marvin_ros2/urdf/marvin_m6s_ccs_696_v4.urdf"
)
FIXTURES = Path(__file__).parent / "fixtures/ik"
MODEL_CHAINS = (
    ("marvin_left", MARVIN, "robot_stand", "TCP_Link_L", 7),
    ("marvin_right", MARVIN, "robot_stand", "TCP_Link_R", 7),
    ("piper", FIXTURES / "piper.urdf", "base_link", "gripper_base", 6),
    ("franka", FIXTURES / "franka_fr3.urdf", "fr3_link0", "fr3_link8", 7),
)


def ikpy_chain(path, base, ee):
    """Build a test-only IKPy reference for the exact selected branch."""
    if "ikpy" not in sys.modules:
        spec = importlib.util.find_spec("ikpy")
        if spec is None:
            raise ImportError(
                "IKPy is required for the test-only FK reference"
            )
        # Import only unchanged math modules, not optional analytics startup.
        package = ModuleType("ikpy")
        package.__path__ = list(spec.submodule_search_locations)
        sys.modules["ikpy"] = package
        package.__version__ = importlib.import_module(
            "ikpy._version"
        ).__version__
    chain_module = importlib.import_module("ikpy.chain")
    tree = ET.parse(path).getroot()
    joints = {
        joint.find("child").get("link"): joint
        for joint in tree.findall("joint")
    }
    elements = [ee]
    current = ee
    while current != base:
        joint = joints[current]
        current = joint.find("parent").get("link")
        elements.extend([joint.get("name"), current])
    elements.reverse()
    parsed = chain_module.Chain.from_urdf_file(
        str(path),
        base_elements=elements,
    )
    terminal = elements[-2]
    end = next(
        i for i, link in enumerate(parsed.links) if link.name == terminal
    )
    chain = chain_module.Chain(parsed.links[: end + 1])
    chain.active_links_mask = np.array(
        [link.joint_type != "fixed" for link in chain.links]
    )
    return chain


@pytest.fixture(params=MODEL_CHAINS, ids=lambda case: case[0])
def model_case(request):
    return request.param


@pytest.fixture
def model(model_case):
    _, path, base, ee, count = model_case
    result = ik_module.Ik(str(path), base, ee)
    assert result.num_joints == count
    return result


@pytest.fixture
def reference(model_case):
    _, path, base, ee, _ = model_case
    chain = ikpy_chain(path, base, ee)

    def fk(q):
        full = chain.active_to_full(q, np.zeros(len(chain.links)))
        return chain.forward_kinematics(full)

    return chain, fk


def pose_values(matrix):
    return [
        *matrix[:3, 3],
        *Rotation.from_matrix(matrix[:3, :3]).as_quat(),
    ]


def test_real_robot_chain_fk_and_analytic_jacobian(model, reference):
    chain, fk = reference
    names = [
        link.name
        for link, active in zip(
            chain.links, chain.active_links_mask, strict=True
        )
        if active
    ]
    assert names == model.joint_names
    rng = np.random.default_rng(2026)
    for _ in range(12):
        q = rng.uniform(model.lower, model.upper)
        np.testing.assert_allclose(model.fk(q), fk(q), atol=1e-12)

        def task(x):
            matrix = fk(x)
            return np.r_[matrix[:3, 3], matrix[:3, :3].ravel()]

        numerical = approx_derivative(task, q, method="3-point")
        np.testing.assert_allclose(
            model.task_jacobian(q), numerical, atol=2e-9
        )


@pytest.mark.parametrize("mode", [None, "X", "Y", "Z", "all"])
@pytest.mark.parametrize("regularization", [None, 0.0, 0.02, 0.07])
@pytest.mark.parametrize("with_seed", [False, True])
def test_modes_and_regularization_match_scipy_objective(
    model, reference, mode, regularization, with_seed
):
    _, fk = reference
    q = model.clamp(np.zeros(model.num_joints))
    seed = q if with_seed else None
    target = fk(q + 0.01)
    result = model.solve_pose(pose_values(target), seed, mode, regularization)
    coefficient = (regularization or 0) if with_seed else 0

    def residual(x):
        matrix = fk(x)
        position = matrix[:3, 3] - target[:3, 3]
        if mode == "all":
            value = np.r_[position, (matrix[:3, :3] - target[:3, :3]).ravel()]
        elif mode is None:
            value = position
        else:
            column = "XYZ".index(mode)
            value = np.r_[position, matrix[:3, column] - target[:3, column]]
        return value + coefficient * np.linalg.norm(x - q)

    baseline = least_squares(residual, q, bounds=(model.lower, model.upper))
    cost = 0.5 * np.linalg.norm(residual(result["q"])) ** 2
    assert cost == pytest.approx(result["cost"], abs=1e-12)
    assert cost <= baseline.cost + 1e-8
    solved = fk(result["q"])
    position_error = np.linalg.norm(solved[:3, 3] - target[:3, 3])
    rotation_error = Rotation.from_matrix(
        solved[:3, :3].T @ target[:3, :3]
    ).magnitude()
    # Axis-only objectives can leave a large unrequested rotation. The
    # existing full-rotation gate still applies to X/Y/Z (but not None).
    expected = not (
        position_error > 0.05 or (mode is not None and rotation_error > 0.3)
    )
    assert result["accepted"] == expected
    assert np.all(result["q"] >= model.lower)
    assert np.all(result["q"] <= model.upper)


def test_clamping_rejection_and_clip_option(model):
    raw = np.full(model.num_joints, np.nan)
    raw[0] = np.inf
    assert np.all(np.isfinite(model.clamp(raw)))
    np.testing.assert_array_equal(
        model.clamp(model.lower),
        np.nextafter(model.lower, model.upper),
    )
    target = model.fk(model.clamp(raw))
    values = pose_values(target)
    values[0] += 10
    for clip in (False, True):
        assert not model.solve_pose(values, raw, clip_to_limit=clip)[
            "accepted"
        ]
    rotated = target.copy()
    rotated[:3, :3] = (
        target[:3, :3] @ Rotation.from_euler("z", 1.0).as_matrix()
    )
    result = model.solve_pose(
        pose_values(rotated),
        raw,
        orientation_mode=None,
        regularization_parameter=None,
    )
    assert result["accepted"]


def test_nan_tolerance_preserves_original_comparison(model):
    target = model.fk(model.clamp(np.zeros(model.num_joints)))
    values = pose_values(target)
    values[0] += 10
    result = model.solve_pose(
        values,
        orientation_mode=None,
        position_tolerance=float("nan"),
    )
    assert result["accepted"]


@pytest.mark.parametrize(
    "base,ee,error",
    [
        ("missing", "TCP_Link_L", "base link"),
        ("robot_stand", "missing", "end-effector link"),
        ("robot_stand", "robot_stand", "must differ"),
        ("TCP_Link_L", "TCP_Link_R", "not connected"),
    ],
)
def test_invalid_or_disconnected_chain(base, ee, error):
    with pytest.raises(ValueError, match=error):
        ik_module.Ik(str(MARVIN), base, ee)


def test_intermediate_base_keeps_fixed_tool_transform():
    tree = ET.parse(MARVIN).getroot()
    joint = next(
        joint
        for joint in tree.findall("joint")
        if joint.get("name") == "Joint2_L"
    )
    base = joint.find("parent").get("link")
    model = ik_module.Ik(str(MARVIN), base, "TCP_Link_L")
    assert model.joint_names == [f"Joint{i}_L" for i in range(2, 8)]
    chain = ikpy_chain(MARVIN, base, "TCP_Link_L")
    q = model.clamp(np.zeros(model.num_joints))
    np.testing.assert_allclose(
        model.fk(q),
        chain.forward_kinematics(
            chain.active_to_full(q, np.zeros(len(chain.links)))
        ),
        atol=1e-12,
    )


def test_franka_axial_wrist_does_not_move_unconstrained_joint():
    path = FIXTURES / "franka_fr3.urdf"
    model = ik_module.Ik(str(path), "fr3_link0", "fr3_link8")
    chain = ikpy_chain(path, "fr3_link0", "fr3_link8")
    q = model.clamp(np.zeros(7))
    full = chain.active_to_full(q + 0.01, np.zeros(len(chain.links)))
    target = chain.forward_kinematics(full)
    # Position and tool Z are independent of the terminal wrist joint.
    np.testing.assert_array_equal(
        model.task_jacobian(q)[[0, 1, 2, 5, 8, 11], -1], np.zeros(6)
    )
    assert model.solve_pose(pose_values(target), None, "Z", None)["accepted"]
