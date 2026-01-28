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

import pytorch_kinematics as pk
import torch
from geometry_msgs.msg import Pose
from pytorch_kinematics.ik import PseudoInverseIK
from pytorch_kinematics.transforms import Transform3d

global_logger = logging.getLogger(__name__)


class IkOptimizer:
    def __init__(
        self,
        urdf_path: str,
        base_link: str,
        ee_link: str,
        clip_to_limit: bool = True,
        logger=None,
    ):
        with open(urdf_path, "rb") as fh:
            self.chain = pk.build_serial_chain_from_urdf(
                fh.read(), ee_link, base_link
            )
        self.chain.to(torch.float64)
        self.limits = torch.tensor(
            self.chain.get_joint_limits(), dtype=self.chain.dtype
        ).T

        self.clip_to_limit = clip_to_limit

        if logger is None:
            logger = global_logger
        self.logger = logger

    def get_joint_names(self) -> list[str]:
        return self.chain.get_joint_parameter_names()

    @property
    def num_joints(self) -> int:
        return len(self.chain.get_joint_parameter_names())

    def solve(
        self, target_pose: Pose, seed_state: list[float] | None = None
    ) -> list[float] | None:
        if seed_state is not None:
            assert len(seed_state) == self.num_joints
            seed_state = torch.tensor(seed_state, dtype=self.chain.dtype)
            seed_state = seed_state[None, :]

        ik_solver = PseudoInverseIK(
            self.chain,
            max_iterations=30,
            joint_limits=self.limits,
            early_stopping_any_converged=True,
            early_stopping_no_improvement="all",
            retry_configs=seed_state,
            pos_tolerance=1e-3,
            rot_tolerance=1e-2,
            lr=0.2,
        )
        target_pose = Transform3d(
            rot=torch.tensor(
                [
                    target_pose.orientation.w,
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                ],
                dtype=self.chain.dtype,
            ),
            pos=torch.tensor(
                [
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z,
                ],
                dtype=self.chain.dtype,
            ),
        )

        solution = ik_solver.solve(target_pose)

        if solution.converged_any:
            final_q = solution.solutions[0, 0]
            return final_q.tolist()
        else:
            self.logger.warn(
                "IK failed to converge from the given initial guess."
            )
            return None
