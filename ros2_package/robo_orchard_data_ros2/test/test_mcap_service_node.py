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

import sys
import types
import uuid
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture(autouse=True)
def ros_dependencies(monkeypatch):
    """Isolate lifecycle unit tests from ROS transport and bag storage."""

    class FakeNode:
        def __init__(self, name):
            self.name = name

        def destroy_node(self):
            pass

    class FakeRecorderStatus:
        IDLE = "idle"
        WAITING = "waiting"
        RECORDING = "recording"
        FINALIZING = "finalizing"
        COMPLETED = "completed"
        FAILED = "failed"

        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

        @staticmethod
        def get_fields_and_field_types():
            return {"header": "std_msgs/Header"}

    class FakeHeader:
        @staticmethod
        def get_fields_and_field_types():
            return {"stamp": "builtin_interfaces/Time", "frame_id": "string"}

    def install(name, **attributes):
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    service_type = types.SimpleNamespace(
        Request=types.SimpleNamespace, Response=types.SimpleNamespace
    )
    install("rclpy")
    install(
        "rclpy.node", Node=FakeNode, ParameterDescriptor=types.SimpleNamespace
    )
    install("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=object)
    install(
        "rclpy.clock",
        Clock=types.SimpleNamespace,
        ClockType=types.SimpleNamespace(STEADY_TIME=1),
    )
    install(
        "rclpy.qos",
        QoSProfile=types.SimpleNamespace,
        ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),
        DurabilityPolicy=types.SimpleNamespace(VOLATILE=2),
        HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
    )
    install(
        "rclpy.serialization",
        serialize_message=lambda message: b"serialized",
        deserialize_message=lambda data, message_type: message_type(),
    )
    install("std_msgs.msg", Header=FakeHeader)
    install("std_srvs.srv", Trigger=service_type)
    install("builtin_interfaces.msg", Time=types.SimpleNamespace)
    install(
        "rosbag2_py",
        SequentialWriter=FakeWriter,
        StorageOptions=types.SimpleNamespace,
        ConverterOptions=types.SimpleNamespace,
        TopicMetadata=types.SimpleNamespace,
    )
    install(
        "robo_orchard_data_msg_ros2.msg", RecorderStatus=FakeRecorderStatus
    )
    install("robo_orchard_data_msg_ros2.srv", StartRecording=service_type)
    modules = (
        "robo_orchard_data_ros2.mcap.service_node",
        "robo_orchard_data_ros2.mcap.config",
    )
    for name in modules:
        monkeypatch.delitem(sys.modules, name, raising=False)
    yield
    for name in modules:
        sys.modules.pop(name, None)


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
        self.writes = []

    def open(self, storage, converter):
        Path(storage.uri).mkdir()

    def create_topic(self, metadata):
        pass

    def write(self, topic, data, timestamp):
        self.writes.append((topic, data, timestamp))

    def close(self):
        self.closed = True


def _make_recorder(tmp_path):
    from builtin_interfaces.msg import Time

    from robo_orchard_data_ros2.mcap.service_node import ServiceMcapRecorder

    recorder = ServiceMcapRecorder.__new__(ServiceMcapRecorder)
    recorder._has_started_writing = True
    recorder.writer = FakeWriter()
    recorder.uri = str(tmp_path / "episode")
    recorder.recording_flag = str(
        Path(recorder.uri) / ServiceMcapRecorder.RECORDING_FILE
    )
    recorder._session_wait_topics = {"/camera/image_raw"}
    recorder._cnt = 1
    recorder._min_timestamp = None
    recorder._max_timestamp = None
    recorder._msg_cnt = {}
    recorder.config = types.SimpleNamespace(
        include_patterns=None,
        exclude_patterns=None,
        wait_for_topics={"/camera/image_raw"},
        wait_for_topics_timeout_s=10.0,
        max_cache_size=0,
        max_timestamp_difference_ns=None,
        static_topics=[],
    )
    recorder.logger = FakeLogger()
    recorder.get_logger = lambda: recorder.logger
    recorder._recording_state = "recording"
    recorder._failure_reason = ""
    recorder._wait_started_at = None
    recorder.session_id = str(uuid.uuid4())
    recorder._frame_rate_monitors = {}
    recorder._active_topic_metadata = {}
    recorder._callback_group = object()
    recorder._latched_msgs = {}
    recorder._hint_freq = 4096
    recorder.messages = []
    recorder._status_publisher = types.SimpleNamespace(
        publish=recorder.messages.append,
        topic_name="/mcap_recorder_service/status",
    )
    recorder.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(
            to_msg=lambda: Time(),
            nanoseconds=100,
        )
    )

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
    assert recorder.uri == saved_uri
    assert recorder.messages[-1].data == "completed"
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


def test_wait_timeout_reports_missing_topics_and_closes_writer(
    tmp_path, monkeypatch
):
    import robo_orchard_data_ros2.mcap.service_node as module

    recorder = _make_recorder(tmp_path)
    writer = recorder.writer
    recorder._recording_state = "waiting"
    recorder._wait_started_at = 0.0
    monkeypatch.setattr(module.time, "monotonic", lambda: 11.0)
    recorder._monitor()
    assert writer.closed
    assert recorder.writer is None
    status = recorder.messages[-1]
    assert status.data == "failed"
    assert "/camera/image_raw" in status.failure_reason
    assert status.session_id == recorder.session_id
    recorder._publish_status()
    assert recorder.messages[-1].data == "failed"


def test_stop_while_waiting_cancels_without_completed(tmp_path):
    recorder = _make_recorder(tmp_path)
    recorder._recording_state = "waiting"
    recorder._cnt = 0
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_stop_request(None, response)
    assert response.success
    assert recorder.messages[-1].data == "idle"
    assert all(message.data != "completed" for message in recorder.messages)


def test_close_failure_is_not_reported_as_success(tmp_path):
    recorder = _make_recorder(tmp_path)

    def fail_close():
        raise OSError("disk error")

    recorder.writer.close = fail_close
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_stop_request(None, response)
    assert not response.success
    assert recorder.messages[-1].data == "failed"
    assert "disk error" in recorder.messages[-1].failure_reason


@pytest.mark.parametrize("fail_write", [False, True])
def test_first_raw_write_reports_recording_or_failure(tmp_path, fail_write):
    from robo_orchard_data_ros2.mcap.config import TopicSpec

    recorder = _make_recorder(tmp_path)
    recorder._recording_state = "waiting"
    recorder._cnt = 0
    recorder._session_wait_topics.clear()

    def write(*args):
        if fail_write:
            raise OSError("disk full")

    recorder.writer.write = write
    recorder._write_raw_message_internal(
        b"data",
        "/camera",
        "/camera",
        TopicSpec(stamp_type="recorder_clock"),
        False,
        None,
    )
    assert recorder._cnt == (0 if fail_write else 1)
    assert recorder.messages[-1].data == (
        "failed" if fail_write else "recording"
    )
    if fail_write:
        assert "disk full" in recorder.messages[-1].failure_reason


def test_start_generates_new_uuid_and_marks_waiting(tmp_path, monkeypatch):
    import robo_orchard_data_ros2.mcap.service_node as module

    recorder = _make_recorder(tmp_path)
    recorder._handle_stop_request(
        None, types.SimpleNamespace(success=None, message=None)
    )
    old_id = recorder.session_id
    destination = tmp_path / "new_episode"

    class Writer(FakeWriter):
        def open(self, storage, converter):
            Path(storage.uri).mkdir()

    monkeypatch.setattr(module.rosbag2_py, "SequentialWriter", Writer)
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_start_request(
        types.SimpleNamespace(destination=str(destination)), response
    )
    assert response.success
    assert recorder.session_id != old_id
    assert uuid.UUID(recorder.session_id).version == 4
    assert recorder.messages[-1].data == "waiting"
    assert recorder.messages[-1].destination == str(destination)
    assert response.message == f"Session initialized at {destination}"
    assert recorder.session_id not in response.message
    assert (destination / recorder.RECORDING_FILE).exists()


def test_status_snapshot_preserves_identity_and_sorts_waiting_topics(tmp_path):
    recorder = _make_recorder(tmp_path)
    recorder._session_wait_topics = {"/z_camera", "/a_camera"}
    recorder._publish_status()
    recorder._publish_status()
    first, second = recorder.messages
    assert first is not second
    assert first.data == second.data == "recording"
    assert first.session_id == second.session_id == recorder.session_id
    assert first.destination == second.destination == recorder.uri
    assert first.waiting_topics == ["/a_camera", "/z_camera"]
    assert first.failure_reason == ""


def test_busy_start_preserves_session_and_writer(tmp_path):
    recorder = _make_recorder(tmp_path)
    identity = (recorder.session_id, recorder.uri, recorder.writer)
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_start_request(
        types.SimpleNamespace(destination=str(tmp_path / "other")), response
    )
    assert not response.success
    assert (recorder.session_id, recorder.uri, recorder.writer) == identity
    assert recorder.messages == []


@pytest.mark.parametrize("failure_step", ["open", "create_topic", "marker"])
def test_start_failure_closes_writer_and_retains_session(
    tmp_path, monkeypatch, failure_step
):
    import robo_orchard_data_ros2.mcap.service_node as module

    recorder = _make_recorder(tmp_path)
    recorder._cleanup_writer()
    recorder._active_topic_metadata = {
        "/camera": types.SimpleNamespace(name="/camera")
    }
    writer = FakeWriter()

    def fail(*args, **kwargs):
        raise OSError("initialization failed")

    if failure_step == "marker":
        monkeypatch.setattr("builtins.open", fail)
    else:
        monkeypatch.setattr(writer, failure_step, fail)
    monkeypatch.setattr(module.rosbag2_py, "SequentialWriter", lambda: writer)
    destination = str(tmp_path / "next")
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_start_request(
        types.SimpleNamespace(destination=destination), response
    )
    assert not response.success
    assert writer.closed == (failure_step != "open")
    assert recorder.writer is None
    assert recorder._cnt == 0
    assert recorder.messages[-1].data == "failed"
    assert recorder.messages[-1].destination == destination
    assert recorder.messages[-1].session_id == recorder.session_id
    assert "initialization failed" in recorder.messages[-1].failure_reason


def test_failed_start_does_not_remove_an_existing_sessions_marker(tmp_path):
    recorder = _make_recorder(tmp_path)
    recorder._cleanup_writer()
    destination = tmp_path / "occupied"
    destination.mkdir()
    marker = destination / recorder.RECORDING_FILE
    marker.touch()
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_start_request(
        types.SimpleNamespace(destination=str(destination)), response
    )
    assert not response.success
    assert marker.exists()


@pytest.mark.parametrize("destination_kind", ["directory", "file", "symlink"])
def test_existing_destination_is_rejected_before_allocating_writer(
    tmp_path, monkeypatch, destination_kind
):
    import robo_orchard_data_ros2.mcap.service_node as module

    recorder = _make_recorder(tmp_path)
    recorder._cleanup_writer()
    destination = tmp_path / "occupied"
    if destination_kind == "directory":
        destination.mkdir()
        metadata = destination / "metadata.yaml"
        metadata.write_text("existing bag metadata")
    elif destination_kind == "file":
        destination.write_text("existing file")
    else:
        destination.symlink_to(tmp_path / "missing")
    allocations = []

    def allocate():
        allocations.append(True)
        return FakeWriter()

    monkeypatch.setattr(module.rosbag2_py, "SequentialWriter", allocate)
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_start_request(
        types.SimpleNamespace(destination=str(destination)), response
    )
    assert not response.success
    assert not allocations
    assert recorder.writer is None
    assert recorder.messages[-1].data == "failed"
    if destination_kind == "directory":
        assert metadata.read_text() == "existing bag metadata"
    elif destination_kind == "file":
        assert destination.read_text() == "existing file"
    else:
        assert destination.is_symlink()


def test_relative_start_destination_is_published_as_absolute(
    tmp_path, monkeypatch
):
    recorder = _make_recorder(tmp_path)
    recorder._cleanup_writer()
    monkeypatch.chdir(tmp_path)
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_start_request(
        types.SimpleNamespace(destination="relative_episode"), response
    )
    assert response.success
    assert recorder.messages[-1].destination == str(
        tmp_path / "relative_episode"
    )
    assert recorder._cnt == 0
    assert recorder._recording_state == "waiting"


@pytest.mark.parametrize("close_fails", [False, True])
def test_marker_removal_failure_cannot_report_completion(
    tmp_path, monkeypatch, close_fails
):
    import robo_orchard_data_ros2.mcap.service_node as module

    recorder = _make_recorder(tmp_path)

    def fail_remove(path):
        raise PermissionError("marker denied")

    def fail_close():
        raise OSError("close failed")

    monkeypatch.setattr(module.os, "remove", fail_remove)
    if close_fails:
        recorder.writer.close = fail_close
    response = types.SimpleNamespace(success=None, message=None)
    recorder._handle_stop_request(None, response)
    assert not response.success
    assert recorder.writer is None
    assert recorder.messages[-1].data == "failed"
    assert "marker denied" in recorder.messages[-1].failure_reason
    if close_fails:
        assert "close failed" in recorder.messages[-1].failure_reason


@pytest.mark.parametrize("elapsed", [9.99, 10.0])
def test_wait_timeout_also_bounds_first_write_with_no_required_topics(
    tmp_path, monkeypatch, elapsed
):
    import robo_orchard_data_ros2.mcap.service_node as module

    recorder = _make_recorder(tmp_path)
    recorder._recording_state = "waiting"
    recorder._session_wait_topics.clear()
    recorder._wait_started_at = 0.0
    recorder._cnt = 0
    monkeypatch.setattr(module.time, "monotonic", lambda: elapsed)
    recorder._monitor()
    assert recorder._recording_state == (
        "waiting" if elapsed < 10.0 else "failed"
    )


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_wait_timeout_must_be_positive_and_finite(timeout):
    from pydantic import ValidationError

    from robo_orchard_data_ros2.mcap.config import RecordConfig

    with pytest.raises(ValidationError):
        RecordConfig(wait_for_topics_timeout_s=timeout)


@pytest.mark.parametrize("fail_write", [False, True])
def test_typed_write_transitions_only_after_success(tmp_path, fail_write):
    from robo_orchard_data_ros2.mcap.config import TopicSpec

    recorder = _make_recorder(tmp_path)
    recorder._recording_state = "waiting"
    recorder._cnt = 0

    def write(*args):
        if fail_write:
            raise OSError("typed write failed")

    recorder.writer.write = write
    recorder._write_message_internal(
        object(), "/camera", "/camera", TopicSpec()
    )
    assert recorder._cnt == (0 if fail_write else 1)
    assert recorder._recording_state == (
        "failed" if fail_write else "recording"
    )


def test_latched_write_failure_stops_the_triggering_message(tmp_path):
    from robo_orchard_data_ros2.mcap.config import TopicSpec

    recorder = _make_recorder(tmp_path)
    recorder._has_started_writing = False
    recorder._recording_state = "waiting"
    recorder._cnt = 0
    recorder._session_wait_topics.clear()
    recorder._latched_msgs = {"/static": ([object()], "/static", TopicSpec())}

    def fail_write(*args):
        raise OSError("latched write failed")

    recorder.writer.write = fail_write
    recorder._message_callback(object(), "/camera", "/camera", TopicSpec())
    assert recorder._cnt == 0
    assert not recorder._has_started_writing
    assert recorder.writer is None
    assert recorder._recording_state == "failed"


def test_discovery_does_not_record_its_own_status(tmp_path):
    from robo_orchard_data_ros2.mcap.config import TopicSpec

    recorder = _make_recorder(tmp_path)
    recorder._insepct_topics = set()
    recorder._subscribers = {}
    recorder.topic_filter = lambda topic: True
    recorder.get_topic_spec = lambda topic: TopicSpec()
    recorder.create_subscription = lambda *args, **kwargs: object()
    recorder.get_topic_names_and_types = lambda: [
        (
            recorder._status_publisher.topic_name,
            ["robo_orchard_data_msg_ros2/msg/RecorderStatus"],
        )
    ]
    recorder.scan_topics()
    assert recorder._active_topic_metadata == {}
    assert recorder._subscribers == {}
    assert recorder.logger.errors == []


def test_dynamic_topic_registration_failure_closes_session(tmp_path):
    from robo_orchard_data_ros2.mcap.config import TopicSpec

    recorder = _make_recorder(tmp_path)
    writer = recorder.writer
    recorder._insepct_topics = set()
    recorder._subscribers = {}
    recorder.topic_filter = lambda topic: True
    recorder.get_topic_names_and_types = lambda: [
        ("/camera", ["std_msgs/msg/Header"])
    ]
    recorder.get_topic_spec = lambda topic: TopicSpec()
    recorder.create_subscription = lambda *args, **kwargs: object()

    def fail_registration(metadata):
        raise OSError("registration failed")

    writer.create_topic = fail_registration
    recorder.scan_topics()
    assert writer.closed
    assert recorder.writer is None
    assert recorder._recording_state == "failed"
    assert "registration failed" in recorder._failure_reason


def test_shutdown_closes_writer_without_publishing(tmp_path):
    recorder = _make_recorder(tmp_path)
    writer = recorder.writer
    marker = Path(recorder.recording_flag)

    def fail_publish(message):
        raise RuntimeError("ROS context shut down")

    recorder._status_publisher.publish = fail_publish
    recorder.destroy_node()
    assert writer.closed
    assert recorder.writer is None
    assert not marker.exists()


def test_startup_and_heartbeat_publish_idle_in_shared_callback_group(
    monkeypatch,
):
    import robo_orchard_data_ros2.mcap.service_node as module

    messages, timers, services = [], [], []
    node_type = module.ServiceMcapRecorder
    monkeypatch.setattr(
        node_type,
        "_initialize_config",
        lambda node: setattr(
            node,
            "config",
            types.SimpleNamespace(
                no_discovery=False,
                wait_for_topics_timeout_s=10.0,
                include_patterns=None,
                exclude_patterns=None,
            ),
        ),
    )
    monkeypatch.setattr(
        node_type, "get_logger", lambda node: FakeLogger(), raising=False
    )
    monkeypatch.setattr(
        node_type,
        "get_clock",
        lambda node: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None)
        ),
        raising=False,
    )
    monkeypatch.setattr(
        node_type,
        "create_publisher",
        lambda node, message_type, topic, depth: types.SimpleNamespace(
            publish=messages.append
        ),
        raising=False,
    )
    monkeypatch.setattr(
        node_type,
        "create_timer",
        lambda node, period, callback, **kwargs: timers.append(
            (period, callback, kwargs)
        ),
        raising=False,
    )
    monkeypatch.setattr(
        node_type,
        "create_service",
        lambda node, service_type, name, callback, **kwargs: services.append(
            (name, kwargs)
        ),
        raising=False,
    )
    recorder = node_type()
    assert messages[-1].data == "idle"
    assert messages[-1].session_id == ""
    assert messages[-1].destination == ""
    assert timers[0][0] == 1.0
    timers[0][1]()
    assert len(messages) == 2
    assert all(
        options["callback_group"] is recorder._callback_group
        for _, _, options in timers
    )
    assert all(
        options["callback_group"] is recorder._callback_group
        for _, options in services
    )
    lifecycle_timers = [
        timer
        for timer in timers
        if timer[1] in (recorder._publish_status, recorder._monitor)
    ]
    assert len(lifecycle_timers) == 2
    for _, _, options in lifecycle_timers:
        assert options["clock"].clock_type == module.ClockType.STEADY_TIME
    recorder.writer = FakeWriter()
    recorder._recording_state = "waiting"
    recorder._wait_started_at = 0.0
    monkeypatch.setattr(module.time, "monotonic", lambda: 11.0)
    for _, callback, _ in lifecycle_timers:
        callback()
    assert messages[-1].data == "failed"
    assert messages[-1].header.stamp is None


def test_unsupported_writer_binding_is_rejected_before_node_setup(monkeypatch):
    import robo_orchard_data_ros2.mcap.service_node as module

    monkeypatch.delattr(module.rosbag2_py.SequentialWriter, "close")
    with pytest.raises(RuntimeError, match=r"rosbag2_py >= 0\.15\.14"):
        module.ServiceMcapRecorder()
