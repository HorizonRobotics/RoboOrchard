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

# ruff: noqa: I001

import sys
import types
import atexit
import builtins
import json
import os
import stat
from contextlib import nullcontext
from pathlib import Path

import pytest


def _install_stub_modules():
    version = types.ModuleType("robo_orchard_inference_app.version")
    version.__version__ = "0.0.0"
    version.__full_version__ = "0.0.0"
    version.__git_hash__ = "test"
    sys.modules.setdefault("robo_orchard_inference_app.version", version)

    st = types.ModuleType("streamlit")
    st.session_state = types.SimpleNamespace()
    st.toast = lambda *args, **kwargs: None
    st.cache_resource = lambda func: func
    st.rerun = lambda: None
    st.dialog = lambda *args, **kwargs: lambda func: func
    sys.modules.setdefault("streamlit", st)
    st_components = types.ModuleType("streamlit.components")
    st_components_v1 = types.ModuleType("streamlit.components.v1")
    st_components_v1.iframe = lambda *args, **kwargs: None
    sys.modules.setdefault("streamlit.components", st_components)
    sys.modules.setdefault("streamlit.components.v1", st_components_v1)

    polling2 = types.ModuleType("polling2")
    polling2.TimeoutException = RuntimeError
    polling2.poll = lambda *args, **kwargs: None
    sys.modules.setdefault("polling2", polling2)

    roslibpy = types.ModuleType("roslibpy")

    class FakeTimeoutError(Exception):
        pass

    class FakeService:
        def __init__(self, client, name, service_type):
            self.client = client
            self.name = name
            self.service_type = service_type

        def call(self, request, timeout=5.0):
            service_results = getattr(self.client, "service_results", {})
            return service_results.get(
                self.name, {"success": True, "message": "ok"}
            )

    class FakeServiceRequest(dict):
        def __init__(self, data=None):
            super().__init__(data or {})

    roslibpy.Service = FakeService
    roslibpy.ServiceRequest = FakeServiceRequest
    roslibpy.Topic = object
    roslibpy.core = types.SimpleNamespace(RosTimeoutError=FakeTimeoutError)
    roslibpy.Ros = object
    sys.modules.setdefault("roslibpy", roslibpy)

    streamlit_tags = types.ModuleType("streamlit_tags")
    streamlit_tags.st_tags = lambda *args, **kwargs: []
    sys.modules.setdefault("streamlit_tags", streamlit_tags)


_install_stub_modules()
sys.path.insert(0, "python/robo_orchard_inference_app")

from robo_orchard_inference_app.components.main_control import (  # noqa: E402
    MainControlComponent,
)
from robo_orchard_inference_app.state import (  # noqa: E402
    CollectingState,
    InferenceState,
)


class FakeRosHelper:
    def __init__(
        self,
        disable_inference_result=True,
        inference_node_active=True,
    ):
        self.calls = []
        self.disable_inference_result = disable_inference_result
        self.inference_node_active = inference_node_active

    def is_inference_node_active(self):
        return self.inference_node_active

    def disable_inference(self):
        self.calls.append("disable_inference")
        return self.disable_inference_result

    def reset_arm(self):
        self.calls.append("reset_arm")

    def recorder_stop_result(self, session_id, destination):
        return None


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def _build_component(
    is_inference_service_running: bool,
    control_mode: str = "auto",
    is_recording: bool = False,
    disable_inference_result: bool = True,
    inference_node_active: bool = True,
):
    component = MainControlComponent.__new__(MainControlComponent)
    component._pending_stop_session_id = None
    component._pending_stop_completed = False
    component.ros_helper = FakeRosHelper(
        disable_inference_result=disable_inference_result,
        inference_node_active=inference_node_active,
    )
    collecting_state = CollectingState(
        inference_state=InferenceState(
            control_mode=control_mode,
            is_inference_service_running=is_inference_service_running,
        ),
        is_recording=is_recording,
    )
    logger = FakeLogger()
    MainControlComponent.collecting_state = property(
        lambda _self: collecting_state
    )
    MainControlComponent.logger = property(lambda _self: logger)
    return component


def test_reset_disables_inference_then_resets_when_node_active():
    component = _build_component(
        is_inference_service_running=True,
        inference_node_active=True,
    )

    component.reset_arm_ctrl_callback()

    assert component.ros_helper.calls == ["disable_inference", "reset_arm"]


def test_reset_skips_disable_when_no_inference_node():
    component = _build_component(
        is_inference_service_running=False,
        inference_node_active=False,
    )

    component.reset_arm_ctrl_callback()

    assert component.ros_helper.calls == ["reset_arm"]


def test_reset_aborts_when_disable_inference_fails():
    component = _build_component(
        is_inference_service_running=True,
        inference_node_active=True,
        disable_inference_result=False,
    )

    component.reset_arm_ctrl_callback()

    assert component.ros_helper.calls == ["disable_inference"]
    assert len(component.logger.warnings) == 1


def test_reset_is_disabled_in_takeover_mode():
    component = _build_component(
        is_inference_service_running=False,
        control_mode="takeover",
    )

    assert component._is_reset_disabled() is True


def test_reset_remains_disabled_while_recording():
    component = _build_component(
        is_inference_service_running=False,
        is_recording=True,
    )

    assert component._is_reset_disabled() is True


@pytest.mark.parametrize(
    ("status", "destination", "success", "terminal", "expected_count"),
    [
        ("recording", "/episode", True, "completed", 1),
        ("waiting", "/episode", True, "completed", 1),
        ("waiting", "/episode", True, "idle", 0),
        ("recording", "/episode", True, "failed", 0),
        ("recording", "/another_episode", True, "completed", 0),
        ("recording", "/episode", False, "completed", 0),
        ("completed", "/episode", True, "completed", 0),
        (None, "/episode", True, "completed", 0),
    ],
)
def test_recorder_stop_counts_only_confirmed_owned_sessions(
    monkeypatch, status, destination, success, terminal, expected_count
):
    component = _build_component(is_inference_service_running=False)
    component.collecting_state.current_data_uri = "/episode"
    component.collecting_state.recording_session_id = "session"
    calls = []
    finalized = []
    snapshot = {
        "data": status,
        "destination": destination,
        "session_id": "session",
    }
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot),
        stop_recording=lambda **kwargs: calls.append("stop") or success,
        recorder_stop_result=lambda *args: None,
    )
    monkeypatch.setattr(
        CollectingState, "at_stop_recording", lambda self: finalized.append(1)
    )
    component.logger.info = lambda message: None
    component.logger.error = lambda message: None

    component._stop_recording_callback()
    assert len(calls) == int(status in {"waiting", "recording"})
    assert not finalized
    component._finalize_stopped_episode(None)
    snapshot["data"] = terminal
    component._finalize_stopped_episode(snapshot)
    component._finalize_stopped_episode(snapshot)
    assert len(finalized) == expected_count
    assert component._pending_stop_session_id is None


def test_other_recorder_session_cannot_finalize_pending_episode(monkeypatch):
    component = _build_component(is_inference_service_running=False)
    component._pending_stop_session_id = "stopped-session"
    component.collecting_state.current_data_uri = "/episode"
    finalized = []
    monkeypatch.setattr(
        CollectingState, "at_stop_recording", lambda self: finalized.append(1)
    )
    component._finalize_stopped_episode(
        {
            "data": "completed",
            "destination": "/episode",
            "session_id": "another-session",
        }
    )
    assert not finalized
    assert component._pending_stop_session_id is None


def test_relative_workspace_produces_absolute_recorder_destination(
    tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    state = CollectingState()
    state.episode_meta.user_name = "operator"
    state.episode_meta.task_name = "task"
    state.prepare(".workspace/")
    destination = Path(state.prepare_recording_path())
    assert destination.is_absolute()
    assert destination.is_relative_to(tmp_path / ".workspace")
    assert destination.parent.is_dir()


def test_failed_start_restores_episode_paths(tmp_path, monkeypatch):
    component = _build_component(is_inference_service_running=False)
    state = component.collecting_state
    state.episode_meta.user_name = "operator"
    state.episode_meta.task_name = "task"
    state.prepare(str(tmp_path))
    state.current_data_uri = "previous-data"
    state.current_log_uri = "previous-log"
    state.recording_session_id = "previous-session"
    component.logger.error = lambda message: None
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: {"data": "idle"},
        start_recording=lambda uri: False,
    )
    component._start_recording_callback()
    assert state.current_data_uri == "previous-data"
    assert state.current_log_uri == "previous-log"
    assert state.recording_session_id == "previous-session"
    assert state.episode_counter.current() == 0
    assert not state.is_recording
    assert not state.recording_start_pending


def test_start_discovery_exception_restores_paths_with_real_helper(
    tmp_path, monkeypatch
):
    import robo_orchard_inference_app.ros_bridge as bridge

    from robo_orchard_inference_app.config import ROSBridgeCfg

    component = _build_component(is_inference_service_running=False)
    state = component.collecting_state
    state.episode_meta.user_name = "operator"
    state.episode_meta.task_name = "task"
    state.prepare(str(tmp_path))
    state.current_data_uri = "previous-data"
    state.current_log_uri = "previous-log"
    component.logger.error = lambda message: None

    def discover():
        raise bridge.roslibpy.core.RosTimeoutError("discovery timeout")

    helper = bridge.RosServiceHelper(
        types.SimpleNamespace(is_connected=True, get_services=discover),
        ROSBridgeCfg(),
        state.inference_state,
        component.logger,
    )
    component.ros_helper = helper
    monkeypatch.setattr(
        helper, "status_snapshot", lambda key: {"data": "idle"}
    )
    try:
        component._start_recording_callback()
        assert state.current_data_uri == "previous-data"
        assert state.current_log_uri == "previous-log"
        assert not state.is_recording
        assert not state.recording_start_pending
    finally:
        helper.cleanup()
        atexit.unregister(helper.cleanup)


def _recording_sidebar(monkeypatch, state):
    import robo_orchard_inference_app.components.sidebar as module

    sidebar = module.SideBarComponent()
    logger = types.SimpleNamespace(
        warn=lambda message: None, error=lambda message: None
    )
    monkeypatch.setattr(
        module.SideBarComponent,
        "collecting_state",
        property(lambda sidebar: state),
    )
    monkeypatch.setattr(
        module.SideBarComponent, "logger", property(lambda sidebar: logger)
    )
    monkeypatch.setattr(
        module.st,
        "spinner",
        lambda *args, **kwargs: nullcontext(),
        raising=False,
    )
    return sidebar


@pytest.mark.parametrize("previous_successes", [0, 1])
def test_deleting_cancelled_session_does_not_subtract_successes(
    tmp_path, monkeypatch, previous_successes
):
    component = _build_component(is_inference_service_running=False)
    state = component.collecting_state
    state.episode_meta.user_name = "operator"
    state.episode_meta.task_name = "task"
    if previous_successes:
        completed = tmp_path / "completed"
        completed.mkdir()
        state.current_data_uri = str(completed)
        state.at_stop_recording()
    cancelled = tmp_path / "cancelled"
    cancelled.mkdir()
    state.current_data_uri = str(cancelled)
    state.recording_session_id = "cancelled"
    snapshot = {
        "data": "waiting",
        "destination": str(cancelled),
        "session_id": "cancelled",
    }
    component.logger.info = lambda message: None
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot),
        stop_recording=lambda **kwargs: True,
        recorder_stop_result=lambda *args: None,
    )
    component._stop_recording_callback()
    snapshot["data"] = "idle"
    component._finalize_stopped_episode(snapshot)
    assert state.episode_counter.current() == previous_successes
    _recording_sidebar(monkeypatch, state)._delete_callback(str(cancelled))
    assert not cancelled.exists()
    assert state.episode_counter.current() == previous_successes


def test_deleting_counted_episode_adjusts_its_original_counter_once(
    tmp_path, monkeypatch
):
    state = CollectingState()
    state.episode_meta.user_name = "first-user"
    state.episode_meta.task_name = "task"
    episode = tmp_path / "episode"
    episode.mkdir()
    state.current_data_uri = str(episode)
    original_counter = state.episode_counter
    state.at_stop_recording()
    state.at_stop_recording()
    assert original_counter.current() == 1
    state.episode_meta.user_name = "other-user"
    other_counter = state.episode_counter
    other_counter.add()
    sidebar = _recording_sidebar(monkeypatch, state)
    sidebar._delete_callback(str(episode))
    assert original_counter.current() == 0
    assert other_counter.current() == 1
    state.at_delete_recording(str(episode))
    assert original_counter.current() == 0


def test_failed_deletion_keeps_counted_episode(tmp_path, monkeypatch):
    import robo_orchard_inference_app.components.sidebar as module

    state = CollectingState()
    episode = tmp_path / "episode"
    episode.mkdir()
    state.current_data_uri = str(episode)
    state.at_stop_recording()

    def fail_remove(uri):
        raise FileNotFoundError(uri)

    monkeypatch.setattr(module, "remove_path", fail_remove)
    _recording_sidebar(monkeypatch, state)._delete_callback(str(episode))
    assert state.episode_counter.current() == 1


@pytest.mark.parametrize(
    "failure_stage", ["create", "write", "close", "replace"]
)
@pytest.mark.parametrize("existing_metadata", [False, True])
def test_metadata_failure_is_atomic_and_retryable(
    start_component, monkeypatch, failure_stage, existing_metadata
):
    import robo_orchard_inference_app.state as module

    state = start_component.collecting_state
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    metadata = episode / "episode_meta.json"
    unrelated = episode / ".episode_meta.unrelated.tmp"
    unrelated.write_text("another writer", encoding="utf-8")
    previous_metadata = '{"instruction": "previous"}'
    if existing_metadata:
        metadata.write_text(previous_metadata, encoding="utf-8")

    def fail_io(*args, **kwargs):
        raise OSError("metadata storage unavailable")

    def temporary_file(*args, **kwargs):
        if failure_stage == "create":
            fail_io()
        handle = builtins.open(*args, **kwargs)
        if failure_stage == "write":
            original_write = handle.write

            def fail_write(payload):
                original_write(payload[:8])
                fail_io()

            handle.write = fail_write
        elif failure_stage == "close":
            original_close = handle.close

            def fail_close():
                original_close()
                fail_io()

            handle.close = fail_close
        return handle

    with monkeypatch.context() as patch:
        patch.setattr(module, "open", temporary_file, raising=False)
        if failure_stage == "replace":
            patch.setattr(module.os, "replace", fail_io)
        with pytest.raises(OSError, match="storage unavailable"):
            state.at_stop_recording()

    assert state.episode_counter.current() == 0
    assert state.current_data_uri not in state._counted_episodes
    expected_files = {unrelated}
    if existing_metadata:
        expected_files.add(metadata)
        assert metadata.read_text(encoding="utf-8") == previous_metadata
    assert set(episode.iterdir()) == expected_files
    assert unrelated.read_text(encoding="utf-8") == "another writer"

    state.at_stop_recording()
    assert state.episode_counter.current() == 1
    assert json.loads(metadata.read_text(encoding="utf-8")) == (
        state.episode_meta.model_dump()
    )
    monkeypatch.setattr(module, "open", fail_io, raising=False)
    state.at_stop_recording()
    assert state.episode_counter.current() == 1


@pytest.mark.parametrize("umask", [0o022, 0o002, 0o077])
@pytest.mark.parametrize("existing_mode", [None, 0o600, 0o640])
def test_metadata_preserves_legacy_content_and_permissions(
    start_component, umask, existing_mode
):
    state = start_component.collecting_state
    state.episode_meta.instruction = "测试 instruction"
    state.episode_meta.tf_directory = "calibration"
    state.episode_meta.metas = {"tags": ["test"]}
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    metadata = episode / "episode_meta.json"
    if existing_mode is not None:
        metadata.write_text("previous metadata", encoding="utf-8")
        metadata.chmod(existing_mode)

    previous_umask = os.umask(umask)
    try:
        state.at_stop_recording()
    finally:
        os.umask(previous_umask)

    expected_mode = (
        existing_mode if existing_mode is not None else 0o666 & ~umask
    )
    assert stat.S_IMODE(metadata.stat().st_mode) == expected_mode
    assert metadata.read_bytes() == state.episode_meta.model_dump_json(
        indent=4
    ).encode("utf-8")
    assert json.loads(metadata.read_text(encoding="utf-8")) == {
        "user_name": "operator",
        "task_name": "task",
        "instruction": "测试 instruction",
        "tf_directory": "calibration",
        "metas": {"tags": ["test"]},
    }
    assert list(episode.iterdir()) == [metadata]
    assert state.episode_counter.current() == 1


def test_metadata_temp_collision_preserves_unowned_file(
    start_component, monkeypatch
):
    import robo_orchard_inference_app.state as module

    state = start_component.collecting_state
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    temporary = episode / ".episode_meta.collision.tmp"
    temporary.write_text("another writer", encoding="utf-8")

    with monkeypatch.context() as patch:
        patch.setattr(
            module, "uuid4", lambda: types.SimpleNamespace(hex="collision")
        )
        with pytest.raises(FileExistsError):
            state.at_stop_recording()

    assert temporary.read_text(encoding="utf-8") == "another writer"
    assert list(episode.iterdir()) == [temporary]
    assert state.episode_counter.current() == 0
    state.at_stop_recording()
    assert state.episode_counter.current() == 1
    assert temporary.exists()


def test_missing_episode_directory_does_not_count_success(start_component):
    state = start_component.collecting_state
    episode = Path(state.prepare_recording_path())

    with pytest.raises(FileNotFoundError):
        state.at_stop_recording()

    assert state.episode_counter.current() == 0
    assert state.current_data_uri not in state._counted_episodes
    assert not episode.exists()
    episode.mkdir()
    state.at_stop_recording()
    assert state.episode_counter.current() == 1
    assert (episode / "episode_meta.json").exists()


def test_metadata_retry_preserves_original_configuration_and_counter(
    start_component,
):
    state = start_component.collecting_state
    state.episode_meta.instruction = "original instruction"
    state.episode_meta.tf_directory = "original tf"
    state.episode_meta.metas = {"tags": ["original"]}
    original_metadata = state.episode_meta.model_dump()
    original_counter = state.episode_counter
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    metadata = episode / "episode_meta.json"
    metadata.mkdir()

    with pytest.raises(OSError):
        state.at_stop_recording()
    state.episode_meta.user_name = "another operator"
    state.episode_meta.task_name = "another task"
    state.episode_meta.instruction = "another instruction"
    state.episode_meta.tf_directory = "another tf"
    state.episode_meta.metas["tags"].append("another tag")
    next_counter = state.episode_counter
    metadata.rmdir()
    state.at_stop_recording()

    assert (
        json.loads(metadata.read_text(encoding="utf-8")) == original_metadata
    )
    assert original_counter.current() == 1
    assert next_counter.current() == 0
    assert state._pending_episode is None


@pytest.mark.parametrize("attempted_settlement", [False, True])
@pytest.mark.parametrize(
    "deleted_episode", ["current", "unrelated", "missing"]
)
def test_deletion_cancels_only_the_deleted_episodes_pending_settlement(
    start_component,
    monkeypatch,
    recording_clock,
    attempted_settlement,
    deleted_episode,
):
    component = start_component
    state = component.collecting_state
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    metadata = episode / "episode_meta.json"
    metadata.mkdir()
    state.recording_session_id = "owned-session"
    snapshot = {
        "data": "recording",
        "session_id": state.recording_session_id,
        "destination": str(episode),
    }
    starts = []
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot),
        stop_recording=lambda **kwargs: True,
        recorder_stop_result=lambda *args: None,
        start_recording=lambda uri: starts.append(uri) or True,
    )
    component._stop_recording_callback()
    snapshot["data"] = "completed"
    if attempted_settlement:
        assert component._finalize_stopped_episode(snapshot) is False

    delete_uri = episode
    if deleted_episode == "unrelated":
        delete_uri = episode.parent / "unrelated"
        delete_uri.mkdir()
    elif deleted_episode == "missing":
        import robo_orchard_inference_app.components.sidebar as module

        def fail_remove(uri):
            raise FileNotFoundError(uri)

        monkeypatch.setattr(module, "remove_path", fail_remove)
    _recording_sidebar(monkeypatch, state)._delete_callback(str(delete_uri))

    recording_clock[0] = "2026_09_16-12_00_01"
    component._start_recording_callback()
    if deleted_episode == "current":
        assert not episode.exists()
        assert state._pending_episode is None
        assert state.recording_session_id is None
        assert component._pending_stop_session_id is None
        assert component._pending_stop_completed is False
        assert len(starts) == 1
        assert starts[0] != str(episode)
    else:
        assert episode.exists()
        assert state._pending_episode is not None
        assert state.recording_session_id == "owned-session"
        assert component._pending_stop_session_id == "owned-session"
        assert not starts
        metadata.rmdir()
        assert component._finalize_stopped_episode(snapshot) is True
        assert state.episode_counter.current() == 1
        return
    assert state.episode_counter.current() == 0


@pytest.mark.parametrize("later_status", ["completed", "foreign", None])
def test_failed_settlement_preserves_completion_and_blocks_start(
    start_component, monkeypatch, recording_clock, later_status
):
    import robo_orchard_inference_app.state as module

    component = start_component
    state = component.collecting_state
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    previous_paths = state.current_data_uri, state.current_log_uri
    state.recording_session_id = "owned-session"
    snapshot = {
        "data": "recording",
        "session_id": state.recording_session_id,
        "destination": state.current_data_uri,
    }
    starts, stops, errors = [], [], []
    component.logger.error = errors.append
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot) if snapshot else None,
        stop_recording=lambda **kwargs: stops.append("stop") or True,
        recorder_stop_result=lambda *args: None,
        start_recording=lambda uri: starts.append(uri) or True,
    )
    component._stop_recording_callback()
    snapshot["data"] = "completed"

    def fail_replace(*args):
        raise OSError("metadata storage unavailable")

    with monkeypatch.context() as patch:
        patch.setattr(module.os, "replace", fail_replace)
        assert component._finalize_stopped_episode(snapshot) is False
        component._update_recorder_state(snapshot)
        assert state.is_recording is False
        assert component._pending_stop_completed is True
        if later_status == "foreign":
            snapshot.update(
                session_id="foreign-session", destination="/foreign-episode"
            )
        elif later_status is None:
            snapshot = None
        assert component._finalize_stopped_episode(snapshot) is False
        component._start_recording_callback()

    assert not starts
    assert stops == ["stop"]
    assert component._pending_stop_session_id == "owned-session"
    assert (state.current_data_uri, state.current_log_uri) == previous_paths
    assert state.episode_counter.current() == 0
    assert errors and all("storage unavailable" in error for error in errors)
    assert not (episode / "episode_meta.json").exists()

    assert component._finalize_stopped_episode(snapshot) is True
    assert component._finalize_stopped_episode(snapshot) is True
    assert component._pending_stop_session_id is None
    assert component._pending_stop_completed is False
    assert state.episode_counter.current() == 1
    assert (episode / "episode_meta.json").exists()
    snapshot = {"data": "idle"}
    recording_clock[0] = "2026_09_16-12_00_01"
    component._start_recording_callback()
    assert len(starts) == 1
    assert starts[0] != previous_paths[0]
    assert state.episode_counter.current() == 1


def test_start_with_unknown_status_preserves_pending_stop():
    component = _build_component(is_inference_service_running=False)
    component._pending_stop_session_id = "pending-session"
    component.collecting_state.recording_session_id = "pending-session"
    component.logger.error = lambda message: None
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: None,
        recorder_stop_result=lambda *args: None,
    )
    component._start_recording_callback()
    assert component._pending_stop_session_id == "pending-session"


def test_start_settles_pending_completion_before_replacing_path(
    tmp_path, monkeypatch
):
    component = _build_component(is_inference_service_running=False)
    state = component.collecting_state
    state.episode_meta.user_name = "operator"
    state.episode_meta.task_name = "task"
    state.prepare(str(tmp_path))
    state.current_data_uri = "previous-data"
    component._pending_stop_session_id = "previous-session"
    state.recording_session_id = "previous-session"
    component.logger.info = lambda message: None
    finalized = []
    monkeypatch.setattr(
        CollectingState,
        "at_stop_recording",
        lambda state: finalized.append(state.current_data_uri),
    )

    def start(uri):
        assert finalized == ["previous-data"]
        assert uri != "previous-data"
        return True

    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: {
            "data": "completed",
            "session_id": "previous-session",
            "destination": "previous-data",
        },
        start_recording=start,
        recorder_stop_result=lambda *args: None,
    )
    component._start_recording_callback()
    assert finalized == ["previous-data"]
    assert component._pending_stop_session_id is None


def test_completed_during_stop_is_settled_once(monkeypatch):
    component = _build_component(is_inference_service_running=False)
    component.collecting_state.current_data_uri = "/episode"
    component.collecting_state.recording_session_id = "session"
    component.logger.info = lambda message: None
    snapshot = {
        "data": "recording",
        "session_id": "session",
        "destination": "/episode",
    }
    finalized = []
    monkeypatch.setattr(
        CollectingState, "at_stop_recording", lambda state: finalized.append(1)
    )

    def stop(**kwargs):
        snapshot["data"] = "completed"
        return True

    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot),
        stop_recording=stop,
        recorder_stop_result=lambda *args: None,
    )
    component._stop_recording_callback()
    assert not finalized
    component._finalize_stopped_episode(snapshot)
    component._finalize_stopped_episode(snapshot)
    assert finalized == [1]


@pytest.mark.parametrize(
    ("status", "start_disabled", "stop_disabled", "busy"),
    [
        ("idle", False, True, False),
        ("waiting", True, False, True),
        ("recording", True, False, True),
        ("finalizing", True, True, True),
        ("completed", False, True, False),
        ("failed", False, True, False),
        (None, True, True, True),
    ],
)
@pytest.mark.parametrize("start_pending", [False, True])
@pytest.mark.parametrize("settlement_pending", [False, True])
def test_recorder_panel_projects_status_without_runtime_or_file_polling(
    tmp_path,
    monkeypatch,
    status,
    start_disabled,
    stop_disabled,
    busy,
    start_pending,
    settlement_pending,
):
    import robo_orchard_inference_app.components.main_control as module

    component = _build_component(
        is_inference_service_running=False, is_recording=True
    )
    component.key_prefix = "recorder-test"
    component.collecting_state.recording_start_pending = start_pending
    component.collecting_state.episode_meta.user_name = "operator"
    component.collecting_state.episode_meta.task_name = "task"
    if settlement_pending:
        component._pending_stop_session_id = "previous-session"
        component._pending_stop_completed = True
        component.collecting_state.recording_session_id = "previous-session"

        def fail_settlement(state):
            raise OSError("metadata storage unavailable")

        monkeypatch.setattr(
            CollectingState, "at_stop_recording", fail_settlement
        )
        component.logger.error = lambda message: None
    snapshot = (
        None
        if status is None
        else {
            "data": status,
            "destination": "/episode",
            "session_id": "session",
            "waiting_topics": ["/camera"],
            "failure_reason": "disk full",
        }
    )
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: snapshot,
        recorder_stop_result=lambda *args: None,
    )
    monkeypatch.setattr(
        MainControlComponent,
        "launch_cfg",
        property(
            lambda component: types.SimpleNamespace(
                workspace=str(tmp_path),
                ui_control=types.SimpleNamespace(
                    start_keyboard=None, stop_keyboard=None
                ),
            )
        ),
    )
    buttons, displayed, details = {}, [], []
    monkeypatch.setattr(
        module.st,
        "expander",
        lambda *args, **kwargs: nullcontext(),
        raising=False,
    )
    monkeypatch.setattr(
        module.st,
        "columns",
        lambda count: [nullcontext(), nullcontext()],
        raising=False,
    )
    monkeypatch.setattr(
        module.st,
        "button",
        lambda label, **options: buttons.update({label: options}),
        raising=False,
    )
    monkeypatch.setattr(module.st, "caption", details.append, raising=False)
    monkeypatch.setattr(module.st, "error", details.append, raising=False)
    monkeypatch.setattr(module.st, "rerun", lambda: None, raising=False)
    monkeypatch.setattr(
        module,
        "multi_status_indicator",
        lambda current_status, **kwargs: displayed.append(current_status),
    )
    component._render_recorder_panel()
    assert not displayed
    assert not details
    assert buttons["▶️ Start"]["disabled"] is (
        start_disabled or start_pending or settlement_pending
    )
    assert buttons["⏹️ Stop"]["disabled"] is stop_disabled
    assert component.collecting_state.is_recording is busy
    assert component._is_reset_disabled() is (busy or start_pending)
    assert component.collecting_state.episode_counter.current() == 0


def test_reset_is_disabled_in_stop_mode():
    component = _build_component(
        is_inference_service_running=False,
        control_mode="stop",
    )

    assert component._is_reset_disabled() is True


@pytest.fixture
def recording_clock(monkeypatch):
    import robo_orchard_inference_app.state as module

    clock = ["2026_09_16-12_00_00"]
    monkeypatch.setattr(module, "time_str_now", lambda: clock[0])
    return clock


@pytest.fixture
def start_component(tmp_path, recording_clock):
    component = _build_component(is_inference_service_running=False)
    state = component.collecting_state
    state.episode_meta.user_name = "operator"
    state.episode_meta.task_name = "task"
    state.prepare(str(tmp_path))
    component.logger.info = lambda message: None
    component.logger.error = lambda message: None
    return component


def test_episode_paths_keep_legacy_layout(start_component, tmp_path):
    state = start_component.collecting_state
    destination = Path(state.prepare_recording_path())
    session = tmp_path / state.session_time_str
    episode_name = "episode_2026_09_16-12_00_00"
    assert destination == session / "data/operator/task" / episode_name
    assert Path(state.current_log_uri) == (
        session / "logs/operator/task" / episode_name
    )
    assert not destination.exists()
    assert not Path(state.current_log_uri).exists()


@pytest.mark.parametrize("root", ["data_root", "log_root"])
@pytest.mark.parametrize("kind", ["directory", "file", "symlink"])
def test_episode_path_collision_does_not_change_current_paths(
    start_component, root, kind
):
    state = start_component.collecting_state
    state.current_data_uri = "previous-data"
    state.current_log_uri = "previous-log"
    collision = Path(getattr(state, root)) / "episode_2026_09_16-12_00_00"
    if kind == "directory":
        collision.mkdir()
    elif kind == "file":
        collision.write_text("existing", encoding="utf-8")
    else:
        collision.symlink_to(collision.parent / "missing")

    with pytest.raises(FileExistsError):
        state.prepare_recording_path()

    assert state.current_data_uri == "previous-data"
    assert state.current_log_uri == "previous-log"
    assert os.path.lexists(collision)


def test_backwards_clock_cannot_reuse_episode_timestamp(
    start_component, recording_clock
):
    state = start_component.collecting_state
    destination = state.prepare_recording_path()
    recording_clock[0] = "2026_09_16-11_59_59"
    with pytest.raises(FileExistsError):
        state.prepare_recording_path()
    assert state.current_data_uri == destination
    recording_clock[0] = "2026_09_16-12_00_01"
    assert state.prepare_recording_path() != destination


def test_start_locks_controls_before_rpc_and_until_status(start_component):
    component = start_component
    state = component.collecting_state
    requests = []

    def start(uri):
        requests.append(uri)
        assert state.recording_start_pending
        assert state.recording_controls_locked
        assert not state.is_recording
        component.reset_arm_ctrl_callback()
        return True

    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: {"data": "idle"},
        start_recording=start,
    )
    component._start_recording_callback()
    component._update_recorder_state(None)
    component._start_recording_callback()
    component.reset_arm_ctrl_callback()

    assert len(requests) == 1
    assert state.recording_start_pending
    assert not state.is_recording
    assert component._is_reset_disabled()
    assert len(component.logger.warnings) == 3


def test_same_second_retry_cannot_accept_previous_failed_status(
    start_component, recording_clock
):
    component = start_component
    state = component.collecting_state
    previous_path = state.prepare_recording_path()
    previous_log = state.current_log_uri
    previous_status = {
        "data": "failed",
        "destination": previous_path,
        "session_id": "previous-session",
    }
    starts = []
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(previous_status),
        start_recording=lambda uri: starts.append(uri) or True,
    )

    component._start_recording_callback()
    assert not starts
    assert state.current_data_uri == previous_path
    assert state.current_log_uri == previous_log
    assert not state.recording_start_pending
    assert "next second" in component.logger.warnings[0]

    recording_clock[0] = "2026_09_16-12_00_01"
    component._start_recording_callback()
    component._update_recorder_state(previous_status)
    component.reset_arm_ctrl_callback()

    assert state.current_data_uri != previous_path
    assert starts == [state.current_data_uri]
    assert (
        Path(state.current_log_uri).name == Path(state.current_data_uri).name
    )
    assert state.recording_start_pending
    assert state.recording_controls_locked
    assert not state.is_recording
    assert len(component.logger.warnings) == 2


def test_rejected_start_does_not_release_allocated_timestamp(
    start_component, recording_clock
):
    component = start_component
    state = component.collecting_state
    state.current_data_uri = "previous-data"
    state.current_log_uri = "previous-log"
    state.recording_session_id = "previous-session"
    requests = []
    snapshot = {"data": "idle"}
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot),
        start_recording=lambda uri: requests.append(uri) or False,
    )

    component._start_recording_callback()
    snapshot.update(
        data="failed", destination=requests[0], session_id="rejected-session"
    )
    component._start_recording_callback()

    assert len(requests) == 1
    assert state.current_data_uri == "previous-data"
    assert state.current_log_uri == "previous-log"
    assert state.recording_session_id == "previous-session"
    assert not state.recording_start_pending

    recording_clock[0] = "2026_09_16-12_00_01"
    component.ros_helper.start_recording = lambda uri: (
        requests.append(uri) or True
    )
    component._start_recording_callback()
    assert len(requests) == 2
    assert requests[0] != requests[1]
    assert state.recording_start_pending
    assert state.recording_session_id is None


@pytest.mark.parametrize(
    "status, destination, session_id",
    [
        ("idle", "/old-episode", "old-session"),
        ("completed", "/other-episode", "other-session"),
        ("idle", "/episode", ""),
        ("unexpected", "/episode", "session"),
    ],
)
def test_unrelated_status_cannot_release_start_guard(
    start_component, status, destination, session_id
):
    component = start_component
    state = component.collecting_state
    state.current_data_uri = "/episode"
    state.recording_start_pending = True

    component._update_recorder_state(
        {
            "data": status,
            "destination": destination,
            "session_id": session_id,
        }
    )

    assert state.recording_start_pending
    assert state.recording_controls_locked


@pytest.mark.parametrize(
    "status, busy",
    [
        ("waiting", True),
        ("recording", True),
        ("finalizing", True),
        ("completed", False),
        ("failed", False),
        ("idle", False),
    ],
)
def test_matching_status_hands_start_guard_over_to_node_state(
    start_component, status, busy
):
    component = start_component
    state = component.collecting_state
    state.current_data_uri = "/episode"
    state.recording_start_pending = True

    assert component._update_recorder_state(
        {
            "data": status,
            "destination": "/episode",
            "session_id": "session",
        }
    )

    assert not state.recording_start_pending
    assert state.is_recording is busy
    assert state.recording_session_id == "session"
    assert state.recording_controls_locked is busy


@pytest.mark.parametrize(
    "control_mode, is_recording, start_pending",
    [
        ("auto", True, False),
        ("auto", False, True),
        ("takeover", False, False),
        ("stop", False, False),
    ],
)
def test_reset_callback_rechecks_controls(
    control_mode, is_recording, start_pending
):
    component = _build_component(
        is_inference_service_running=False,
        control_mode=control_mode,
        is_recording=is_recording,
    )
    component.collecting_state.recording_start_pending = start_pending

    component.reset_arm_ctrl_callback()

    assert component.ros_helper.calls == []
    assert len(component.logger.warnings) == 1


@pytest.mark.parametrize("status_before_reply", [False, True])
@pytest.mark.parametrize("outcome", ["unknown", "exception"])
def test_uncertain_start_binds_status_and_settles_after_stop(
    start_component, status_before_reply, outcome
):
    component = start_component
    state = component.collecting_state
    state.current_data_uri = "previous-data"
    state.current_log_uri = "previous-log"
    state.recording_session_id = "previous-session"
    messages = {"current": {"data": "idle"}}
    starts, stops = [], []

    def start(uri):
        starts.append(uri)
        Path(uri).mkdir()
        messages["recording"] = {
            "data": "recording",
            "session_id": "started-session",
            "destination": uri,
        }
        messages["current"] = (
            dict(messages["recording"]) if status_before_reply else None
        )
        if outcome == "exception":
            raise TimeoutError("Start reply lost")
        return None

    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: messages["current"],
        start_recording=start,
        stop_recording=lambda **kwargs: stops.append(True) or True,
        recorder_stop_result=lambda *args: None,
    )

    component._start_recording_callback()

    assert state.current_data_uri == starts[0]
    assert state.current_log_uri != "previous-log"
    assert state.recording_start_pending is not status_before_reply
    assert state.recording_controls_locked
    assert state.recording_session_id == (
        "started-session" if status_before_reply else None
    )
    if not status_before_reply:
        component._start_recording_callback()
        assert len(starts) == 1
    messages["current"] = dict(messages["recording"])
    component._stop_recording_callback()
    assert stops == [True]
    assert state.recording_session_id == "started-session"
    assert component._pending_stop_session_id == "started-session"

    completed = {**messages["recording"], "data": "completed"}
    component._finalize_stopped_episode(completed)
    component._finalize_stopped_episode(completed)

    assert state.episode_counter.current() == 1
    assert (Path(state.current_data_uri) / "episode_meta.json").is_file()
    assert component._pending_stop_session_id is None


def test_stop_cannot_settle_another_session_at_the_same_destination(
    start_component,
):
    component = start_component
    state = component.collecting_state
    state.current_data_uri = "/episode"
    state.recording_session_id = "owned-session"
    snapshot = {
        "data": "recording",
        "destination": "/episode",
        "session_id": "foreign-session",
    }
    component.ros_helper = types.SimpleNamespace(
        status_snapshot=lambda key: dict(snapshot),
        stop_recording=lambda **kwargs: True,
        recorder_stop_result=lambda *args: None,
    )

    component._stop_recording_callback()
    component._finalize_stopped_episode({**snapshot, "data": "completed"})

    assert state.recording_session_id == "owned-session"
    assert component._pending_stop_session_id is None
    assert state.episode_counter.current() == 0


@pytest.mark.parametrize("outcome", ["accepted", "timeout", "rejected"])
@pytest.mark.parametrize("later", ["completed", "another_session", "offline"])
def test_owned_stop_settles_received_completion_despite_lost_reply_or_status(
    start_component, monkeypatch, outcome, later
):
    import robo_orchard_inference_app.ros_bridge as bridge
    from robo_orchard_inference_app.config import ROSBridgeCfg

    component = start_component
    state = component.collecting_state
    episode = Path(state.prepare_recording_path())
    episode.mkdir()
    state.recording_session_id = "owned-session"
    state.is_recording = True
    client = types.SimpleNamespace(
        is_connected=True,
        get_services=lambda: ["/mcap_recorder_service/stop_recording"],
        off=lambda *args: None,
    )
    helper = bridge.RosServiceHelper(
        client, ROSBridgeCfg(), state.inference_state, component.logger
    )
    helper._status_topics = [types.SimpleNamespace(unsubscribe=lambda: None)]
    component.ros_helper = helper
    owned = {
        "data": "recording",
        "session_id": "owned-session",
        "destination": str(episode),
    }
    helper._receive_status("recorder", owned)
    requests = []

    def call(request, timeout):
        requests.append(request)
        assert component._pending_stop_session_id == "owned-session"
        helper._receive_status("recorder", {**owned, "data": "completed"})
        if later == "another_session":
            helper._receive_status(
                "recorder",
                {
                    "data": "recording",
                    "session_id": "another-session",
                    "destination": "/another-episode",
                },
            )
        elif later == "offline":
            client.is_connected = False
            helper._invalidate_status()
        if outcome == "timeout":
            raise bridge.roslibpy.core.RosTimeoutError("Stop reply lost")
        return {"success": outcome == "accepted"}

    monkeypatch.setattr(
        bridge.roslibpy,
        "Service",
        lambda *args: types.SimpleNamespace(call=call),
    )
    try:
        component._stop_recording_callback()
        snapshot = helper.status_snapshot("recorder")
        component._finalize_stopped_episode(snapshot)
        component._finalize_stopped_episode(snapshot)
        assert requests == [{}]
        assert component._pending_stop_session_id is None
        assert state.episode_counter.current() == int(outcome != "rejected")
        assert (episode / "episode_meta.json").exists() is (
            outcome != "rejected"
        )
    finally:
        helper.cleanup()
        atexit.unregister(helper.cleanup)


def test_foreign_session_without_completion_cannot_discard_pending_stop(
    start_component,
):
    component = start_component
    state = component.collecting_state
    state.current_data_uri = "/episode"
    state.recording_session_id = "owned-session"
    component._pending_stop_session_id = "owned-session"

    assert (
        component._finalize_stopped_episode(
            {
                "data": "completed",
                "session_id": "another-session",
                "destination": "/another-episode",
            }
        )
        is False
    )
    assert component._pending_stop_session_id == "owned-session"
    assert state.episode_counter.current() == 0
