# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
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

from collections import Counter
from pathlib import Path
from typing import Annotated, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, StringConstraints

__all__ = [
    "CommandChannel",
    "ControlManagerConfig",
    "load_control_manager_config",
]

NonEmptyString = Annotated[
    str,
    StringConstraints(strip_whitespace=True, min_length=1),
]
FullNodeName = Annotated[
    str,
    StringConstraints(
        strip_whitespace=True,
        pattern=r"^(/[A-Za-z_][A-Za-z0-9_]*)+$",
    ),
]


class CommandChannel(BaseModel):
    """One autonomous/override command path managed as a unit.

    The channel describes only ROS transport wiring. Action decoding and
    model response fields remain owned by the Deploy configuration.
    """

    model_config = ConfigDict(extra="forbid")

    name: NonEmptyString
    kind: Literal["joint_command"] = "joint_command"
    msg_type: NonEmptyString = "sensor_msgs/msg/JointState"
    autonomous_topic: NonEmptyString
    override_topic: NonEmptyString
    output_topic: NonEmptyString


class ControlManagerConfig(BaseModel):
    """Validated runtime wiring for one Control Manager instance."""

    model_config = ConfigDict(extra="forbid")

    channels: list[CommandChannel] = Field(min_length=1)
    enable_services: list[NonEmptyString] = Field(default_factory=list)
    reset_services: list[NonEmptyString] = Field(default_factory=list)
    inference_disable_services: list[NonEmptyString] = Field(
        default_factory=list
    )
    inference_node_candidates: list[FullNodeName] = Field(
        default_factory=list,
        description=(
            "Alternative fully qualified inference node names. When set, "
            "reset checks current discovery without waiting and skips "
            "inference disable only if no candidate node or configured "
            "disable service is discovered. When empty, all configured "
            "disable services remain mandatory."
        ),
    )
    replay_time_s: float = Field(default=2.0, ge=0.0)
    status_publish_rate_hz: float = Field(default=1.0, gt=0.0)
    service_wait_timeout_s: float = Field(default=5.0, gt=0.0)
    service_response_timeout_s: float = Field(default=60.0, gt=0.0)

    def model_post_init(self, context: object) -> None:
        """Validate channel identities and inference reset wiring."""
        if (
            self.inference_node_candidates
            and not self.inference_disable_services
        ):
            raise ValueError(
                "inference_node_candidates requires "
                "inference_disable_services."
            )
        names = [channel.name for channel in self.channels]
        duplicate_names = sorted(
            name for name, count in Counter(names).items() if count > 1
        )
        if duplicate_names:
            raise ValueError(f"Duplicate channel names: {duplicate_names}.")

        outputs = [channel.output_topic for channel in self.channels]
        duplicate_outputs = sorted(
            topic for topic, count in Counter(outputs).items() if count > 1
        )
        if duplicate_outputs:
            raise ValueError(f"Duplicate output topics: {duplicate_outputs}.")


def load_control_manager_config(
    config_file: str | Path,
) -> ControlManagerConfig:
    """Load and validate a Control Manager YAML or JSON file.

    Args:
        config_file: Path supplied through the node's ``config_file``
            parameter.

    Returns:
        The validated, standalone Control Manager configuration.

    Raises:
        FileNotFoundError: If ``config_file`` does not exist.
        ValueError: If the file cannot be parsed or fails validation.
    """
    path = Path(config_file)
    if not path.is_file():
        raise FileNotFoundError(
            f"Control Manager config file does not exist: {path}"
        )

    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(
            f"Failed to parse Control Manager config '{path}': {exc}"
        ) from exc

    return ControlManagerConfig.model_validate(data)
