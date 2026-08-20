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

import inspect
import logging
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

global_logger = logging.getLogger(__name__)
IKPY_REGULARIZATION_WITH_SEED = 0.02

# How far the solved pose may sit from the requested one before the solution
# is discarded. The position figure is deliberately looser than it looks: at
# the edge of the workspace ikpy lands a few centimetres short while keeping
# orientation almost exact, and a tighter gate rejects those solves -- which
# leaves the arm with no command at all and freezes it mid-motion. Measured on
# a run driven into the boundary on purpose: every rejected solve was 3.1-4.4
# cm out with under 1.8 deg of orientation error. Tracking slightly behind the
# operator degrades far more gracefully than stopping dead.
DEFAULT_POSITION_TOLERANCE = 0.05
DEFAULT_ORIENTATION_TOLERANCE = 0.3


def _resolve_urdf_chain(
    urdf_path: str, base_link: str, ee_link: str
) -> tuple[list[str], str]:
    root = ET.parse(urdf_path).getroot()
    link_names = {
        link.attrib["name"]
        for link in root.findall("link")
        if "name" in link.attrib
    }
    if base_link not in link_names:
        raise ValueError(f"URDF base link '{base_link}' does not exist")
    if ee_link not in link_names:
        raise ValueError(f"URDF end-effector link '{ee_link}' does not exist")
    if base_link == ee_link:
        raise ValueError("URDF base link and end-effector link must differ")

    joints_by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        if child is None or parent is None:
            continue
        child_link = child.attrib.get("link")
        if child_link:
            joints_by_child[child_link] = joint

    reversed_elements = [ee_link]
    current_link = ee_link
    visited_links = set()
    terminal_joint_name = ""
    while current_link != base_link:
        if current_link in visited_links:
            raise ValueError(
                f"URDF contains a cycle while resolving '{ee_link}'"
            )
        visited_links.add(current_link)

        joint = joints_by_child.get(current_link)
        if joint is None:
            raise ValueError(
                f"URDF link '{ee_link}' is not connected to "
                f"base link '{base_link}'"
            )
        parent = joint.find("parent")
        if parent is None or "link" not in parent.attrib:
            raise ValueError(
                f"URDF joint '{joint.attrib.get('name', '')}' has no parent"
            )

        joint_name = joint.attrib.get("name")
        if not joint_name:
            raise ValueError("URDF joint on selected chain has no name")
        if not terminal_joint_name:
            terminal_joint_name = joint_name
        parent_link = parent.attrib["link"]
        reversed_elements.extend([joint_name, parent_link])
        current_link = parent_link

    return list(reversed(reversed_elements)), terminal_joint_name


class IkOptimizer:
    def __init__(
        self,
        urdf_path: str,
        base_link: str,
        ee_link: str,
        clip_to_limit: bool = True,
        regularization_parameter: float | None = IKPY_REGULARIZATION_WITH_SEED,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
        orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE,
        logger=None,
    ):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            from ikpy.chain import Chain

            base_elements, terminal_joint_name = _resolve_urdf_chain(
                urdf_path, base_link, ee_link
            )
            parsed_chain = Chain.from_urdf_file(
                urdf_path,
                base_elements=base_elements,
                active_links_mask=None,
                last_link_vector=None,
                name=ee_link,
            )
            terminal_index = next(
                (
                    index
                    for index, link in enumerate(parsed_chain.links)
                    if link.name == terminal_joint_name
                ),
                None,
            )
            if terminal_index is None:
                raise ValueError(
                    f"IK chain for '{ee_link}' does not contain terminal "
                    f"joint '{terminal_joint_name}'"
                )
            if terminal_index == len(parsed_chain.links) - 1:
                self.chain = parsed_chain
            else:
                self.chain = Chain(
                    parsed_chain.links[: terminal_index + 1],
                    name=ee_link,
                )

        self._active_mask = []
        self._active_indices = []
        for i, link in enumerate(self.chain.links):
            is_active = (
                hasattr(link, "joint_type") and link.joint_type != "fixed"
            )
            self._active_mask.append(is_active)
            if is_active:
                self._active_indices.append(i)
        self.chain.active_links_mask = self._active_mask

        self.clip_to_limit = clip_to_limit
        self.regularization_parameter = regularization_parameter
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance

        if logger is None:
            logger = global_logger
        self.logger = logger

        if self.regularization_parameter is not None:
            self._verify_regularization_support()

    def get_joint_names(self) -> list[str]:
        return [self.chain.links[i].name for i in self._active_indices]

    @property
    def num_joints(self) -> int:
        return len(self._active_indices)

    @staticmethod
    def _clip_value(value: float, bounds) -> float:
        if bounds is None:
            return float(value)

        lo, hi = bounds
        if lo is not None:
            value = max(value, lo)
        if hi is not None:
            value = min(value, hi)
        return float(value)

    def _clamp_seed(self, seed: list[float]) -> list[float]:
        full = [0.0] * len(self.chain.links)
        for j, idx in enumerate(self._active_indices):
            full[idx] = self._clip_value(seed[j], self.chain.links[idx].bounds)
        return full

    def _verify_regularization_support(self):
        try:
            signature = inspect.signature(self.chain.inverse_kinematics_frame)
        except (TypeError, ValueError):
            return

        if "regularization_parameter" in signature.parameters:
            return

        if any(
            parameter.kind == inspect.Parameter.VAR_KEYWORD
            for parameter in signature.parameters.values()
        ):
            return

        raise RuntimeError(
            "The installed ikpy does not support regularization_parameter"
        )

    def solve(
        self,
        target_pose: Pose,
        seed_state: list[float] | None = None,
        orientation_mode: str | None = "all",
    ) -> list[float] | None:
        quat = [
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ]
        rot = Rotation.from_quat(quat).as_matrix()
        target_matrix = np.eye(4)
        target_matrix[:3, :3] = rot
        target_matrix[:3, 3] = [
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
        ]

        if seed_state is not None:
            if len(seed_state) != self.num_joints:
                raise ValueError(
                    f"seed_state length {len(seed_state)} does not match "
                    f"{self.num_joints} joints"
                )
            initial = self._clamp_seed(seed_state)
        else:
            initial = [0.0] * len(self.chain.links)

        ik_kwargs = {
            "initial_position": initial,
            "orientation_mode": orientation_mode,
        }
        if (
            seed_state is not None
            and self.regularization_parameter is not None
        ):
            ik_kwargs["regularization_parameter"] = (
                self.regularization_parameter
            )

        try:
            result = self.chain.inverse_kinematics_frame(
                target_matrix, **ik_kwargs
            )
        except Exception as e:
            self.logger.warning("IK solver exception: %s" % e)
            return None

        fk_result = self.chain.forward_kinematics(result)
        pos_err = np.linalg.norm(fk_result[:3, 3] - target_matrix[:3, 3])
        rot_err = Rotation.from_matrix(
            fk_result[:3, :3].T @ target_matrix[:3, :3]
        ).magnitude()

        if pos_err > self.position_tolerance or (
            # A caller that asked for no orientation constraint gets no
            # orientation check: the solver was told to spend its freedom on
            # position, so scoring the result on the axis it was allowed to
            # ignore rejects work that did exactly what was requested.
            orientation_mode is not None
            and rot_err > self.orientation_tolerance
        ):
            self.logger.warning(
                "IK failed to converge (pos_err=%.4f, rot_err=%.4f)"
                % (pos_err, rot_err)
            )
            return None

        solution = [float(result[i]) for i in self._active_indices]

        if self.clip_to_limit:
            solution = [
                self._clip_value(solution[j], self.chain.links[idx].bounds)
                for j, idx in enumerate(self._active_indices)
            ]

        return solution
