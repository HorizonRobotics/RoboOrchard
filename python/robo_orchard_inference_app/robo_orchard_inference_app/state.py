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
import stat
from contextlib import suppress
from datetime import datetime
from typing import Literal
from uuid import uuid4

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
    control_mode: Literal["auto", "takeover", "stop", "resetting"] | None = (
        None
    )
    is_inference_service_running: bool | None = None


class NotReadyError(Exception):
    pass


class EpisodeMeta(pydantic.BaseModel):
    user_name: str = ""
    task_name: str = ""
    instruction: str = ""
    tf_directory: str = ""
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
        default_factory=InferenceState
    )

    is_recording: bool = False
    recording_start_pending: bool = False
    """A local Start request is awaiting its matching Recorder status."""

    recording_session_id: str | None = None
    """The session observed at this App's unique recording destination."""

    _counted_episodes: dict[str, EpisodeCounter] = pydantic.PrivateAttr(
        default_factory=dict
    )
    _pending_episode: tuple[str, EpisodeMeta, EpisodeCounter] | None = (
        pydantic.PrivateAttr(default=None)
    )
    _last_recording_timestamp: str = pydantic.PrivateAttr(default="")

    @property
    def recording_controls_locked(self) -> bool:
        """Protect recording controls without inferring node runtime state."""
        return self.recording_start_pending or self.is_recording

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

    def prepare(self, workspace: str) -> None:
        if not self.is_configured:
            raise NotReadyError

        session_root = os.path.abspath(
            os.path.join(workspace, self.session_time_str)
        )
        self.data_root = os.path.join(
            session_root, "data", self.user_name, self.task_name
        )
        self.log_root = os.path.join(
            session_root, "logs", self.user_name, self.task_name
        )
        os.makedirs(self.data_root, exist_ok=True)
        os.makedirs(self.log_root, exist_ok=True)

    def prepare_recording_path(self) -> str:
        """Allocate a legacy timestamp-only episode name without reuse.

        Raises:
            NotReadyError: The user and task have not been configured.
            FileExistsError: The timestamp has not advanced or either path
                already exists. No paths are changed on rejection.
        """
        if not self.is_configured:
            raise NotReadyError

        time_str = time_str_now()
        episode_name = f"episode_{time_str}"
        data_uri = os.path.join(self.data_root, episode_name)
        log_uri = os.path.join(self.log_root, episode_name)
        if (
            time_str <= self._last_recording_timestamp
            or os.path.lexists(data_uri)
            or os.path.lexists(log_uri)
        ):
            raise FileExistsError(
                "Episode timestamp is already used; wait for the next "
                "second before starting another recording."
            )

        self._last_recording_timestamp = time_str
        self.current_data_uri = data_uri
        self.current_log_uri = log_uri

        return self.current_data_uri

    def at_stop_recording(self) -> None:
        """Finalize an explicit Stop after Recorder confirms completion.

        Recorder status owns is_recording; heartbeats do not finalize episodes.
        Metadata is replaced atomically before counting. I/O failures propagate
        without marking the episode finalized. Retries retain the first
        attempt's metadata and counter, independently of later UI edits.
        New files honor the process umask; existing file modes are preserved.
        """
        if self.current_data_uri in self._counted_episodes:
            return
        if self._pending_episode is None:
            self._pending_episode = (
                self.current_data_uri,
                self.episode_meta.model_copy(deep=True),
                self.episode_counter,
            )
        data_uri, episode_meta, counter = self._pending_episode

        metadata_path = os.path.join(data_uri, "episode_meta.json")
        temporary_path = os.path.join(
            data_uri, f".episode_meta.{uuid4().hex}.tmp"
        )
        try:
            metadata_stat = os.stat(metadata_path, follow_symlinks=False)
        except FileNotFoundError:
            metadata_stat = None

        metadata_file = open(temporary_path, "x", encoding="utf-8")
        try:
            with metadata_file:
                if metadata_stat and stat.S_ISREG(metadata_stat.st_mode):
                    os.chmod(
                        temporary_path, stat.S_IMODE(metadata_stat.st_mode)
                    )
                metadata_file.write(episode_meta.model_dump_json(indent=4))
            os.replace(temporary_path, metadata_path)
        finally:
            with suppress(FileNotFoundError):
                os.unlink(temporary_path)

        counter.add()
        self._counted_episodes[data_uri] = counter
        self._pending_episode = None

    def at_delete_recording(self, uri: str) -> None:
        """Remove a deleted absolute URI from the counter that counted it.

        Recordings not finalized by this App do not affect its counters.
        Call only after successful deletion; repeated calls are idempotent.
        Deleting the current episode abandons ownership and pending metadata.
        """
        if uri == self.current_data_uri:
            self.recording_session_id = None
        if self._pending_episode and self._pending_episode[0] == uri:
            self._pending_episode = None
        counter = self._counted_episodes.pop(uri, None)
        if counter is not None:
            counter.sub()


class LogMessage(pydantic.BaseModel):
    """Represents a single structured log entry."""

    timestamp: datetime = pydantic.Field(default_factory=datetime.now)
    level: Literal["info", "warning", "error"] = "info"
    message: str


class AppState(pydantic.BaseModel):
    """Root model for all application-wide session state."""

    logs: list[LogMessage] = []
