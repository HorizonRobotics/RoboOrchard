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

import importlib.util
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

RCLPY_AVAILABLE = importlib.util.find_spec("rclpy") is not None
pytestmark = pytest.mark.skipif(
    not RCLPY_AVAILABLE,
    reason="rclpy is required for mcap recorder tests",
)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeWriter:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def _make_recorder(tmp_path):
    from robo_orchard_data_ros2.mcap.service_node import ServiceMcapRecorder

    recorder = ServiceMcapRecorder.__new__(ServiceMcapRecorder)
    recorder._has_started_writing = True
    recorder.writer = FakeWriter()
    recorder.uri = str(tmp_path / "episode")
    recorder.recording_flag = str(
        Path(recorder.uri) / ServiceMcapRecorder.RECORDING_FILE
    )
    recorder._session_wait_topics = {"/camera/image_raw"}
    recorder._cnt = 0
    recorder._min_timestamp = None
    recorder._max_timestamp = None
    recorder._msg_cnt = {}
    recorder.config = types.SimpleNamespace(
        include_patterns=None,
        exclude_patterns=None,
    )
    recorder.logger = FakeLogger()
    recorder.get_logger = lambda: recorder.logger

    Path(recorder.uri).mkdir()
    Path(recorder.recording_flag).touch()
    return recorder


def test_stop_request_closes_writer_and_reports_saved_uri(tmp_path):
    recorder = _make_recorder(tmp_path)
    writer = recorder.writer
    saved_uri = recorder.uri
    recording_flag = recorder.recording_flag
    response = types.SimpleNamespace(success=None, message=None)

    returned = recorder._handle_stop_request(None, response)

    assert returned is response
    assert response.success is True
    assert response.message == f"Stopped. Saved to {saved_uri}"
    assert writer.closed is True
    assert recorder.writer is None
    assert recorder.uri is None
    assert recorder.recording_flag is None
    assert recorder._session_wait_topics == set()
    assert not Path(recording_flag).exists()


def test_stop_request_rejects_when_not_recording(tmp_path):
    recorder = _make_recorder(tmp_path)
    recorder.writer = None
    response = types.SimpleNamespace(success=None, message=None)

    returned = recorder._handle_stop_request(None, response)

    assert returned is response
    assert response.success is False
    assert response.message == "Not recording."
