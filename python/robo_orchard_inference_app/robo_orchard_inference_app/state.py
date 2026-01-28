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

import os
from datetime import datetime
from typing import Literal

import pydantic

from robo_orchard_inference_app.utils import time_str_now


class EpisodeCounter:
    """A simple counter class for tracking episode numbers.

    Attributes:
        idx (int): The current episode count.
    """

    def __init__(self):
        self.idx: int = 0

    def current(self) -> int:
        """Returns the current episode count.

        Returns:
            int: Current episode number
        """
        return self.idx

    def add(self) -> int:
        """Increments the episode count by 1.

        Returns:
            int: New episode number
        """
        self.idx += 1
        return self.idx

    def sub(self) -> int:
        """Decrements the episode count by 1.

        Returns:
            int: New episode number
        """
        self.idx -= 1
        return self.idx


class InferenceState(pydantic.BaseModel):
    control_mode: Literal["auto", "takeover", "stop"] = "auto"
    is_inference_service_running: bool = False
    arm_ctrl_status: Literal["enabled", "disabled"] = "enabled"


class NotReadyError(Exception):
    pass


class NotRecordingError(Exception):
    pass


class EpisodeMeta(pydantic.BaseModel):
    user_name: str = ""
    task_name: str = ""
    instruction: str = ""
    metas: dict[str, list[str]] = pydantic.Field(default_factory=dict)


class CollectingState(pydantic.BaseModel):
    """Manages the state of data collection process."""

    model_config = pydantic.ConfigDict(arbitrary_types_allowed=True)

    session_time_str: str = pydantic.Field(default_factory=time_str_now)
    """Timestamp of the current session."""

    episode_meta: EpisodeMeta = pydantic.Field(default_factory=EpisodeMeta)
    """Episode metas for current session."""

    data_root: str = ""
    """Root directory for data storage."""

    log_root: str = ""
    """Root directory for log storage."""

    current_data_uri: str = ""
    """Current data storage URI."""

    current_log_uri: str = ""
    """Current log storage URI."""

    episode_counters: dict[str, EpisodeCounter] = pydantic.Field(
        default_factory=lambda: dict()
    )
    """Dictionary of episode counters per user-task pair."""

    inference_state: InferenceState = pydantic.Field(
        default_factory=lambda: InferenceState(
            control_mode="auto",
            arm_ctrl_status="enabled",
            is_inference_service_running=False,
        )
    )

    is_recording: bool = False

    @property
    def user_name(self) -> str:
        return self.episode_meta.user_name

    @property
    def task_name(self) -> str:
        return self.episode_meta.task_name

    @property
    def episode_counter(self) -> EpisodeCounter:
        """Gets or creates an episode counter for the current user-task pair.

        Returns:
            EpisodeCounter: Counter instance for the current user and task
        """
        key = "{}:{}".format(self.user_name, self.task_name)
        if key not in self.episode_counters:
            self.episode_counters[key] = EpisodeCounter()
        return self.episode_counters[key]

    @property
    def is_configured(self) -> bool:
        return self.user_name and self.task_name

    def prepare(self, workspace: str):
        if not self.is_configured:
            raise NotReadyError

        session_root = os.path.join(workspace, self.session_time_str)
        self.data_root = os.path.join(
            session_root, "data", self.user_name, self.task_name
        )
        self.log_root = os.path.join(
            session_root, "logs", self.user_name, self.task_name
        )
        os.makedirs(self.data_root, exist_ok=True)
        os.makedirs(self.log_root, exist_ok=True)

    def prepare_recording_path(self) -> str:
        if not self.is_configured:
            raise NotReadyError

        time_str = time_str_now()

        self.current_data_uri = os.path.join(
            self.data_root, f"episode_{time_str}"
        )
        self.current_log_uri = os.path.join(
            self.log_root, f"episode_{time_str}"
        )

        return self.current_data_uri

    def at_start_recording(self):
        self.is_recording = True

    def at_stop_recording(self):
        if not self.is_recording:
            raise NotRecordingError

        self.is_recording = False

        self.episode_counter.add()

        if os.path.exists(self.current_data_uri):
            with open(
                os.path.join(self.current_data_uri, "episode_meta.json"), "w"
            ) as fh:
                fh.write(self.episode_meta.model_dump_json(indent=4))


class LogMessage(pydantic.BaseModel):
    """Represents a single structured log entry."""

    timestamp: datetime = pydantic.Field(default_factory=datetime.now)
    level: Literal["info", "warning", "error"] = "info"
    message: str


class AppState(pydantic.BaseModel):
    """Root model for all application-wide session state."""

    logs: list[LogMessage] = []
