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

from typing import Annotated, List, Literal, Union

from pydantic import BaseModel, ConfigDict, Field, model_validator
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

__all__ = [
    "QosProfile",
    "ObsChannelBase",
    "ImageChannel",
    "CameraInfoChannel",
    "JointStateChannel",
    "ObsChannel",
    "ObservationConfig",
    "ActionChannelBase",
    "JointCommandChannel",
    "ActionChannel",
    "TrajectoryStitchConfig",
    "ControlConfig",
    "DeployConfig",
]


class QosProfile(BaseModel):
    """ROS 2 QoS settings used to create a channel endpoint.

    The defaults match the QoS previously used when an observation channel
    selected the ``default`` preset: keep the last 10 messages, use reliable
    delivery, and do not retain samples for late subscribers.
    """

    depth: int = 10
    reliability: int = ReliabilityPolicy.RELIABLE
    durability: int = DurabilityPolicy.VOLATILE
    history: int = HistoryPolicy.KEEP_LAST


class ObsChannelBase(BaseModel):
    """Fields shared by every observation channel.

    A channel binds one robot-side topic to one model-server-side request
    field. The topic belongs to the embodiment, the key belongs to the
    contract with ``DeployConfig.server_url``; the deploy node only moves
    data between them.
    """

    server_input_key: str = Field(
        description="Request field name agreed with the model server.",
    )
    topic: str = Field(
        description="ROS 2 topic providing this observation.",
    )
    qos_profile: QosProfile = Field(
        default_factory=QosProfile,
        description="ROS 2 subscription QoS settings for this channel.",
    )


class ImageChannel(ObsChannelBase):
    """An image topic decoded through cv_bridge."""

    kind: Literal["image"] = "image"
    msg_type: str = Field(
        default="sensor_msgs/msg/Image",
        description="Declared ROS 2 message type of the topic.",
    )
    encoding: str = Field(
        default="bgr8",
        description="cv_bridge desired_encoding. Depth uses 'passthrough'.",
    )


class CameraInfoChannel(ObsChannelBase):
    """A camera info topic decoded into its 3x4 projection matrix."""

    kind: Literal["camera_info"] = "camera_info"
    msg_type: str = Field(
        default="sensor_msgs/msg/CameraInfo",
        description="Declared ROS 2 message type of the topic.",
    )


class JointStateChannel(ObsChannelBase):
    """A joint state topic decoded into paired ROS names and positions."""

    model_config = ConfigDict(extra="forbid")

    kind: Literal["joint_state"] = "joint_state"
    msg_type: str = Field(
        default="sensor_msgs/msg/JointState",
        description="Declared ROS 2 message type of the topic.",
    )
    joint_names: List[str] | None = Field(
        default=None,
        description=(
            "ROS joints to select in this order. None keeps all names and "
            "positions in the published order."
        ),
    )


ObsChannel = Annotated[
    Union[ImageChannel, CameraInfoChannel, JointStateChannel],
    Field(discriminator="kind"),
]


class ObservationConfig(BaseModel):
    channels: List[ObsChannel] = Field(
        min_length=1,
        description="Observation channels sent to the model server.",
    )
    sync_slop: float = Field(
        default=0.1,
        description="ApproximateTimeSynchronizer slop in seconds.",
    )
    sync_queue_size: int = Field(
        default=1,
        description="ApproximateTimeSynchronizer queue size.",
    )

    @model_validator(mode="after")
    def _validate_server_input_keys(self) -> "ObservationConfig":
        for channel in self.channels:
            if isinstance(channel, JointStateChannel) and (
                channel.server_input_key in {"instruction", "delay_horizon"}
            ):
                raise ValueError(
                    "Joint observation key conflicts with a request form field"
                )
        keys = [channel.server_input_key for channel in self.channels]
        duplicated = sorted({key for key in keys if keys.count(key) > 1})
        if duplicated:
            raise ValueError(
                f"Duplicated server_input_key {duplicated}. Each channel "
                f"must map to a distinct model server request field."
            )
        return self


class ActionChannelBase(BaseModel):
    """Fields shared by every action channel.

    A channel binds one model-server-side response field to one robot-side
    command topic. It is the mirror of :class:`ObsChannelBase`: the key
    belongs to the contract with ``DeployConfig.server_url``, the topic
    belongs to the embodiment.
    """

    server_output_key: str = Field(
        description="Response field name agreed with the model server.",
    )
    topic: str = Field(
        description="ROS 2 topic this action is published on.",
    )


class JointCommandChannel(ActionChannelBase):
    """A joint position command published as a JointState message.

    ``joint_names`` is per channel, so two arms of the same embodiment can
    name their joints differently.
    """

    model_config = ConfigDict(extra="forbid")

    kind: Literal["joint_command"] = "joint_command"
    msg_type: str = Field(
        default="sensor_msgs/msg/JointState",
        description="Declared ROS 2 message type of the topic.",
    )
    server_remaining_key: str | None = Field(
        default=None,
        description=(
            "Request field this channel's not yet published steps are sent "
            "back under, so the server can account for what the robot is "
            "still executing. None sends nothing."
        ),
    )
    joint_names: List[str] = Field(
        min_length=1,
        description="Joint names of this arm or hand, in channel order.",
    )
    velocities: List[float] | None = Field(
        default=None,
        description=(
            "Constant velocity field of the command. None leaves it empty. "
            "Must match the length of joint_names."
        ),
    )
    efforts: List[float] | None = Field(
        default=None,
        description=(
            "Constant effort field of the command. None leaves it empty. "
            "Must match the length of joint_names."
        ),
    )

    @model_validator(mode="after")
    def _validate_lengths(self) -> "JointCommandChannel":
        expected = len(self.joint_names)
        for name, values in (
            ("velocities", self.velocities),
            ("efforts", self.efforts),
        ):
            if values is not None and len(values) != expected:
                raise ValueError(
                    f"Channel '{self.server_output_key}' declares "
                    f"{expected} joints but {len(values)} {name}."
                )
        return self


ActionChannel = Annotated[
    Union[JointCommandChannel],
    Field(discriminator="kind"),
]


class TrajectoryStitchConfig(BaseModel):
    """Parameters of the chunk-handover trajectory solve.

    The motion limits describe the robot and have no defaults. They are an
    envelope, not a shaping target: a limit set below the motion the chunk
    already asks for makes the program hard enough that the solver runs out
    of iterations, and the chunk is installed unchanged. The rest are
    solver settings whose defaults are a starting point, not a tuned value
    for any particular embodiment.
    """

    max_velocity: float = Field(
        gt=0,
        description="Joint velocity envelope imposed on the solved trajectory, in rad/s. Set above the fastest motion the model commands.",  # noqa: E501
    )
    max_acceleration: float = Field(
        gt=0,
        description="Joint acceleration envelope imposed on the solved trajectory, in rad/s^2.",  # noqa: E501
    )
    max_jerk: float = Field(
        gt=0,
        description="Joint jerk envelope imposed on the solved trajectory, in rad/s^3.",  # noqa: E501
    )
    solver_dt: float = Field(
        gt=0,
        default=0.02,
        description="Time step of the solver grid, in seconds. Deliberately coarser than the control period: a grid at control rate multiplies the problem size for no accuracy that survives resampling, and the weights below are relative to this value.",  # noqa: E501
    )
    horizon_quantum: int = Field(
        default=16,
        gt=0,
        description="Round the solved window length up to a multiple of this many control steps, so the horizon takes a few distinct values and their solvers can be reused. Building a solver costs far more than a solve.",  # noqa: E501
    )
    track_weight: float = Field(
        ge=0,
        default=1.0,
        description="Penalty on departing from the requested chunk.",
    )
    terminal_track_weight: float = Field(
        ge=0,
        default=10.0,
        description="Extra penalty on departing from the requested chunk at its final step, so the solved trajectory ends where the chunk ends.",  # noqa: E501
    )
    acceleration_weight: float = Field(
        ge=0,
        default=1e-6,
        description="Penalty on acceleration. Scales against the horizon: a short chunk needs larger accelerations to cover the same distance, so a weight tuned for a long horizon leaves a short one short of its endpoint.",  # noqa: E501
    )
    jerk_weight: float = Field(
        ge=0,
        default=2e-6,
        description="Penalty on jerk, which is what the solve exists to bound.",  # noqa: E501
    )
    terminal_velocity_weight: float = Field(
        ge=0,
        default=0.05,
        description="Damping on the final velocity. The terminal state is inherited verbatim as the next solve's initial condition, so leaving it free lets it ride the bound and compound until the solve is infeasible.",  # noqa: E501
    )
    terminal_acceleration_weight: float = Field(
        ge=0,
        default=0.05,
        description="Damping on the final acceleration, for the same reason as terminal_velocity_weight.",  # noqa: E501
    )


class ControlConfig(BaseModel):
    channels: List[ActionChannel] = Field(
        min_length=1,
        description="Action channels published from the model response.",
    )
    control_frequency: float = Field(
        default=25.0, description="Control frequency in Hz."
    )


class DeployConfig(BaseModel):
    observation_config: ObservationConfig
    control_config: ControlConfig
    server_url: str = Field(
        default="http://localhost:2000/holobrain", description="Server URL."
    )
    infer_frequency: float = Field(
        default=1.0, description="Inference frequency in Hz."
    )
    delay_horizon: int = Field(
        default=10,
        description="Delay horizon (in number of steps) under model frame rate.",  # noqa: E501
    )
    max_delay_horizon: int | None = Field(
        default=None,
        description="Maximum delay horizon (in number of steps) under control frame rate.",  # noqa: E501
    )
    max_command_velocity: float | None = Field(
        default=None,
        gt=0,
        description="Clamp how far a joint command may move from the command before it, in rad/s, applied per action channel. Bounds the published command stream only: the first command of a channel, and the first after execution resumes, have no predecessor and are not bounded. A robot limit, so it has no default; null disables the clamp.",  # noqa: E501
    )
    trajectory_stitch: TrajectoryStitchConfig | None = Field(
        default=None,
        description="Re-solve each incoming action chunk so its position, velocity and acceleration continue the chunk the robot is already executing. Switching chunks is otherwise continuous in position alone, which leaves a velocity step at every handover. Requires the osqp and scipy packages. Null, the default, installs chunks unchanged.",  # noqa: E501
    )

    @model_validator(mode="after")
    def _validate_remaining_request_keys(self) -> "DeployConfig":
        keys = {
            channel.server_input_key
            for channel in self.observation_config.channels
        } | {"instruction", "delay_horizon"}
        for channel in self.control_config.channels:
            key = channel.server_remaining_key
            if key is None:
                continue
            if key in keys:
                raise ValueError(
                    f"server_remaining_key '{key}' conflicts with request key"
                )
            keys.add(key)
        return self
