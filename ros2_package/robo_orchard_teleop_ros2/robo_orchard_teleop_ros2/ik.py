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

import logging
import math

from geometry_msgs.msg import Pose

from robo_orchard_teleop_ros2._ik import Ik

global_logger = logging.getLogger(__name__)
IKPY_REGULARIZATION_WITH_SEED = 0.02

# At the workspace edge, accepting a few centimetres of tracking error is
# preferable to freezing the command stream. Keep the existing pose gates.
DEFAULT_POSITION_TOLERANCE = 0.05
DEFAULT_ORIENTATION_TOLERANCE = 0.3


class IkOptimizer:
    """Solve the selected URDF chain with Pinocchio and bounded TRF.

    Joint names, seeds, and results follow base-to-end-effector URDF order.
    Positions are metres; joint angles and orientation tolerance are radians.
    """

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
    ) -> None:
        self._solver = Ik(urdf_path, base_link, ee_link)
        self.clip_to_limit = clip_to_limit
        self.regularization_parameter = regularization_parameter
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.logger = global_logger if logger is None else logger

    def get_joint_names(self) -> list[str]:
        """Return active joint names in base-to-end-effector order."""
        return list(self._solver.joint_names)

    @property
    def num_joints(self) -> int:
        """Number of active joints in the selected chain."""
        return self._solver.num_joints

    def solve(
        self,
        target_pose: Pose,
        seed_state: list[float] | None = None,
        orientation_mode: str | None = "all",
    ) -> list[float] | None:
        """Return joint radians, or None for solver failures or rejected poses.

        A supplied seed enables the configured scalar IKPy regularization.
        None orientation mode constrains position only and skips the rotation
        gate; X/Y/Z constrain that rotation-matrix column. Input shape and
        invalid quaternion errors remain caller errors.
        """
        quat = [
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ]
        # Previously Rotation.from_quat validated outside the solver exception
        # handler. Keep invalid quaternions as caller errors.
        if not all(math.isfinite(value) for value in quat):
            raise ValueError("Found zero norm quaternions in quat.")
        if math.hypot(*quat) == 0:
            raise ValueError("Found zero norm quaternions in quat.")
        if seed_state is not None and len(seed_state) != self.num_joints:
            raise ValueError(
                f"seed_state length {len(seed_state)} does not match "
                f"{self.num_joints} joints"
            )
        pose = [
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            *quat,
        ]
        try:
            result = self._solver.solve_pose(
                pose,
                seed=seed_state,
                orientation_mode=orientation_mode,
                regularization_parameter=self.regularization_parameter,
                clip_to_limit=self.clip_to_limit,
                position_tolerance=self.position_tolerance,
                orientation_tolerance=self.orientation_tolerance,
            )
        except Exception as error:
            self.logger.warning("IK solver exception: %s" % error)
            return None
        if not result["accepted"]:
            self.logger.warning(
                "IK failed to converge (pos_err=%.4f, rot_err=%.4f)"
                % (result["position_error"], result["rotation_error"])
            )
            return None
        return result["q"]
