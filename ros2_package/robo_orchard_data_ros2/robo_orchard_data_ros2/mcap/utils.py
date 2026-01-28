# Project RoboOrchard
#
# Copyright (c) 2024 Horizon Robotics. All Rights Reserved.
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

import re
from collections import deque


class TopicFilterConflictError(Exception):
    """Exception raised when a topic matches both include and exclude filters."""  # noqa: E501

    pass


class FrameRateMonitor:
    """Monitors the frame rate of a topic based on message timestamps."""  # noqa: E501

    def __init__(self, window_size=30, timeout_threshold: float = 5 * 1e9):
        self.timestamps = deque(maxlen=window_size)
        self.timeout_threshold = timeout_threshold

    def update(self, timestamp_ns: int):
        self.timestamps.append(timestamp_ns)

    def get_fps(self, current_timestamp_ns: int) -> float:
        if (
            len(self.timestamps) < 2
            or current_timestamp_ns - self.timestamps[-1]
            > self.timeout_threshold
        ):
            return 0.0

        time_diff_ns = self.timestamps[-1] - self.timestamps[0]
        if time_diff_ns <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / (time_diff_ns * 1e-9)

    def __len__(self) -> int:
        return len(self.timestamps)


class TopicFilter:
    def __init__(
        self,
        include_patterns: str | list[str],
        exclude_patterns: str | list[str],
    ):
        self.include_topics = set()
        self.include_regex = []
        self.exclude_topics = set()
        self.exclude_regex = []

        if include_patterns:
            for pattern in include_patterns:
                if "*" in pattern or "." in pattern:
                    self.include_regex.append(re.compile(pattern))
                else:
                    self.include_topics.add(pattern)

        if exclude_patterns:
            for pattern in exclude_patterns:
                if "*" in pattern or "." in pattern:
                    self.exclude_regex.append(re.compile(pattern))
                else:
                    self.exclude_topics.add(pattern)

        self._has_topic_filter = (
            bool(self.include_topics)
            or bool(self.include_regex)
            or bool(self.exclude_topics)
            or bool(self.exclude_regex)
        )

    def __call__(self, topic: str) -> bool:
        if not self._has_topic_filter:
            return True

        if self.include_topics or self.include_regex:
            include_match = topic in self.include_topics or any(
                regex.match(topic) for regex in self.include_regex
            )
            if not include_match:
                return False
        else:
            include_match = True

        exclude_match = topic in self.exclude_topics or any(
            regex.match(topic) for regex in self.exclude_regex
        )

        if (
            (self.include_topics or self.include_regex)
            and include_match
            and exclude_match
        ):
            msg = (
                f'Topic "{topic}" matches both include and exclude filters.\n'
                f"Include: {self.include_topics}\n"
                f"Exclude: {self.exclude_topics}"
            )
            raise TopicFilterConflictError(msg)

        if exclude_match:
            return False

        return True
