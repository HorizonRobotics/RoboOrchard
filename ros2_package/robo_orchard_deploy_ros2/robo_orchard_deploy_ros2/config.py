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

from pydantic import BaseModel, Field, model_validator
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
    """A joint state topic decoded into a joint position vector."""

    kind: Literal["joint_state"] = "joint_state"
    msg_type: str = Field(
        default="sensor_msgs/msg/JointState",
        description="Declared ROS 2 message type of the topic.",
    )
    joint_names: List[str] | None = Field(
        default=None,
        description=(
            "Joints to read, in model order. None keeps the order published "
            "by the topic, which is only safe when the publisher order is "
            "known and stable."
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
        description="Joint names of this arm or hand, in model order.",
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
