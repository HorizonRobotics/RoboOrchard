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

"""Check the Hand2 joint-name contract without ROS or SDK imports."""

import ast
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

ROS_ROOT = Path(__file__).resolve().parents[2]
PYTHON_SOURCE = (
    ROS_ROOT
    / "robo_orchard_wuji_glove_ros2"
    / "robo_orchard_wuji_glove_ros2/retarget_node.py"
)
CPP_SOURCE = (
    ROS_ROOT
    / "robo_orchard_wuji2_ros2"
    / "src/wuji_hand2_driver/conversion/joint_mapping.hpp"
)


def _check_mapping(python_source: str, cpp_source: str, side: str) -> None:
    tree = ast.parse(python_source)
    bodies_assignment = next(
        node
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "_HAND2_JOINT_NAME_BODIES"
            for target in node.targets
        )
    )
    bodies = ast.literal_eval(bodies_assignment.value)
    names_expression = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute) and target.attr == "_joint_names"
            for target in node.targets
        )
        and any(
            isinstance(child, ast.Name)
            and child.id == "_HAND2_JOINT_NAME_BODIES"
            for child in ast.walk(node.value)
        )
    )
    # Evaluate only the checked-in naming expression, not the ROS node or SDK.
    python_names = eval(
        compile(ast.Expression(names_expression), str(PYTHON_SOURCE), "eval"),
        {
            "__builtins__": {},
            "_HAND2_JOINT_NAME_BODIES": bodies,
            "self": SimpleNamespace(_hand_side=side),
        },
    )
    table = re.search(r"kJointSpecs\s*\{\{(.*?)\}\};", cpp_source, re.S)
    assert table is not None, "C++ joint table not found"
    entries = re.findall(r'\{\s*(\d+)\s*,\s*"([^"]+)"\s*\}', table[1])
    prefixes = re.search(
        r"prefix\s*=\s*side\s*==\s*HandSide::kLeft\s*\?\s*"
        r'"([^"]+)"\s*:\s*"([^"]+)"',
        cpp_source,
    )
    assert prefixes is not None, "C++ side prefixes not found"
    assert prefixes.groups() == ("l_", "r_")
    prefix = prefixes[1 if side == "left" else 2]
    cpp_names = [prefix + body for _, body in entries]
    count = re.search(r"kJointCount\s*=\s*(\d+)", cpp_source)
    assert count is not None and int(count[1]) == 20
    assert len(bodies) == len(entries) == 20
    assert len(set(python_names)) == len(set(cpp_names)) == 20
    assert len({nid for nid, _ in entries}) == 20
    assert python_names == cpp_names, "Hand2 joint names or order drifted"


@pytest.mark.parametrize("side", ["left", "right"])
def test_hand2_joint_names_order_and_prefix_match_driver(side):
    _check_mapping(PYTHON_SOURCE.read_text(), CPP_SOURCE.read_text(), side)


@pytest.mark.parametrize("mutation", ["name", "order", "prefix"])
def test_mapping_check_rejects_drift(mutation):
    python_source = PYTHON_SOURCE.read_text()
    if mutation == "name":
        python_source = python_source.replace("thumb_cmc_flex", "wrong_joint")
    elif mutation == "order":
        python_source = (
            python_source.replace("thumb_cmc_flex", "temporary_joint")
            .replace("thumb_cmc_abd", "thumb_cmc_flex")
            .replace("temporary_joint", "thumb_cmc_abd")
        )
    else:
        python_source = python_source.replace(
            "self._hand_side[0]", "'wrong_prefix'"
        )
    with pytest.raises(AssertionError, match="drifted"):
        _check_mapping(python_source, CPP_SOURCE.read_text(), "left")
