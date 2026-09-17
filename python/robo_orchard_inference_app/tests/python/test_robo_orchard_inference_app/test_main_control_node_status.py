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

# ruff: noqa: E402, I001

import sys
import types
from contextlib import contextmanager

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
    st.set_page_config = lambda *args, **kwargs: None
    st.expander = lambda *args, **kwargs: _Expander()
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
    roslibpy.Service = object
    roslibpy.ServiceRequest = dict
    roslibpy.Topic = object
    roslibpy.core = types.SimpleNamespace(RosTimeoutError=RuntimeError)
    roslibpy.Ros = object
    sys.modules.setdefault("roslibpy", roslibpy)

    streamlit_tags = types.ModuleType("streamlit_tags")
    streamlit_tags.st_tags = lambda *args, **kwargs: []
    sys.modules.setdefault("streamlit_tags", streamlit_tags)


_install_stub_modules()
sys.path.insert(0, "python/robo_orchard_inference_app")

import streamlit as st  # noqa: E402

from robo_orchard_inference_app.components.main_control import (
    MainControlComponent,
)  # noqa: E402


class _Expander:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, tb):
        return False


def test_is_tf_publisher_online_checks_only_static_tf_node():
    component = object.__new__(MainControlComponent)
    component.ros_helper = types.SimpleNamespace(
        get_node_names=lambda: [
            "/foxglove_bridge",
            "/static_tf_publisher",
            "/robot/inference_service/sync_node",
        ]
    )

    assert component._is_tf_publisher_online() is True


def test_is_tf_publisher_online_returns_false_when_missing():
    component = object.__new__(MainControlComponent)
    component.ros_helper = types.SimpleNamespace(
        get_node_names=lambda: [
            "/foxglove_bridge",
            "/robot/inference_service/sync_node",
        ]
    )

    assert component._is_tf_publisher_online() is False


def _make_ros_helper_stub(startup_ids):
    ids = iter(startup_ids)
    calls = {"invalidate": 0, "startup_id_queries": 0}

    def _get_startup_id():
        calls["startup_id_queries"] += 1
        try:
            return next(ids)
        except StopIteration:
            return None

    def _invalidate():
        calls["invalidate"] += 1

    helper = types.SimpleNamespace(
        get_tf_publisher_startup_id=_get_startup_id,
        invalidate_static_transform_cache=_invalidate,
    )
    return helper, calls


def test_handle_tf_recovery_skips_when_publisher_offline():
    component = object.__new__(MainControlComponent)
    helper, calls = _make_ros_helper_stub(["abc"])
    component.ros_helper = helper
    component._known_tf_publisher_startup_id = None

    component._handle_tf_publisher_recovery(current_online=False)

    assert calls == {"invalidate": 0, "startup_id_queries": 0}
    assert component._known_tf_publisher_startup_id is None


def test_handle_tf_recovery_skips_when_startup_id_unavailable():
    component = object.__new__(MainControlComponent)
    helper, calls = _make_ros_helper_stub([None])
    component.ros_helper = helper
    component._known_tf_publisher_startup_id = "old"

    component._handle_tf_publisher_recovery(current_online=True)

    assert calls == {"invalidate": 0, "startup_id_queries": 1}
    assert component._known_tf_publisher_startup_id == "old"


def test_handle_tf_recovery_invalidates_on_first_contact_and_records_id():
    component = object.__new__(MainControlComponent)
    helper, calls = _make_ros_helper_stub(["abc", "abc"])
    component.ros_helper = helper
    component._known_tf_publisher_startup_id = None

    component._handle_tf_publisher_recovery(current_online=True)
    component._handle_tf_publisher_recovery(current_online=True)

    assert calls == {"invalidate": 1, "startup_id_queries": 2}
    assert component._known_tf_publisher_startup_id == "abc"


def test_handle_tf_recovery_detects_restart_without_seeing_offline():
    component = object.__new__(MainControlComponent)
    helper, calls = _make_ros_helper_stub(["abc", "xyz"])
    component.ros_helper = helper
    component._known_tf_publisher_startup_id = None

    component._handle_tf_publisher_recovery(current_online=True)
    component._handle_tf_publisher_recovery(current_online=True)

    assert calls == {"invalidate": 2, "startup_id_queries": 2}
    assert component._known_tf_publisher_startup_id == "xyz"


def test_handle_tf_recovery_invalidates_once_after_offline_then_online():
    component = object.__new__(MainControlComponent)
    helper, calls = _make_ros_helper_stub(["abc", "xyz", "xyz"])
    component.ros_helper = helper
    component._known_tf_publisher_startup_id = None

    component._handle_tf_publisher_recovery(current_online=True)
    component._handle_tf_publisher_recovery(current_online=False)
    component._handle_tf_publisher_recovery(current_online=True)
    component._handle_tf_publisher_recovery(current_online=True)

    assert calls == {"invalidate": 2, "startup_id_queries": 3}
    assert component._known_tf_publisher_startup_id == "xyz"


def test_render_configure_panel_checks_tf_recovery_before_sync(monkeypatch):
    component = object.__new__(MainControlComponent)
    calls = []
    st.session_state.collecting_state = types.SimpleNamespace(
        episode_meta="episode"
    )
    component.ros_helper = types.SimpleNamespace(
        sync_static_transforms=lambda episode_meta: calls.append(
            ("sync", episode_meta)
        )
    )
    component._configure_panel = lambda: calls.append(
        ("configure_panel", None)
    )
    monkeypatch.setattr(
        st,
        "expander",
        lambda *args, **kwargs: _Expander(),
        raising=False,
    )
    monkeypatch.setattr(component, "_is_tf_publisher_online", lambda: True)
    monkeypatch.setattr(
        component,
        "_handle_tf_publisher_recovery",
        lambda current_online: calls.append(("recovery", current_online)),
    )

    component._render_configure_panel()

    assert calls == [
        ("configure_panel", None),
        ("recovery", True),
        ("sync", "episode"),
    ]


def test_component_initialization_starts_inference_status_monitor(monkeypatch):
    import robo_orchard_inference_app.components.main_control as control

    calls = []
    helper = types.SimpleNamespace(
        start_status_monitor=lambda: calls.append("start_monitor")
    )
    collecting_state = types.SimpleNamespace(
        episode_meta=object(), inference_state=object()
    )
    for name, value in {
        "collecting_state": collecting_state,
        "launch_cfg": types.SimpleNamespace(ros_bridge=object()),
        "ros_client": object(),
        "logger": object(),
    }.items():
        monkeypatch.setattr(
            MainControlComponent,
            name,
            property(lambda self, value=value: value),
        )
    monkeypatch.setattr(
        control, "EditEpisodeMetaComponent", lambda **kwargs: object()
    )
    monkeypatch.setattr(control, "RosServiceHelper", lambda **kwargs: helper)

    component = MainControlComponent()

    assert component.ros_helper is helper
    assert calls == ["start_monitor"]


def test_both_status_panels_are_scheduled_for_periodic_refresh(monkeypatch):
    import robo_orchard_inference_app.components.main_control as control

    component = object.__new__(MainControlComponent)
    panels = [
        "state",
        "configure",
        "recorder",
        "robot_control",
        "handeye_calib",
    ]
    calls = []
    fragments = []
    for panel in panels:
        monkeypatch.setattr(
            component,
            f"_render_{panel}_panel",
            lambda panel=panel: calls.append(panel),
        )

    def fragment(*, run_every):
        def decorate(callback):
            fragments.append((run_every, callback))
            return callback

        return decorate

    monkeypatch.setattr(control.st, "fragment", fragment, raising=False)

    component()

    assert calls == panels
    assert fragments == [
        (1.0, component._render_state_panel),
        (1.0, component._render_recorder_panel),
    ]


@pytest.mark.parametrize(
    "status, color",
    [
        ("idle", "grey"),
        ("waiting", "orange"),
        ("recording", "red"),
        ("finalizing", "orange"),
        ("completed", "green"),
        ("failed", "red"),
        (None, "grey"),
    ],
)
@pytest.mark.parametrize("pending", [None, "start", "stop"])
def test_state_panel_projects_status_before_rendering(
    monkeypatch, status, color, pending
):
    import robo_orchard_inference_app.components.main_control as control

    component = object.__new__(MainControlComponent)
    state = types.SimpleNamespace(
        control_mode="takeover", is_inference_service_running=True
    )
    collecting_state = types.SimpleNamespace(
        inference_state=state, recording_start_pending=pending == "start"
    )
    component._pending_stop_session_id = (
        "owned-session" if pending == "stop" else None
    )
    monkeypatch.setattr(
        MainControlComponent,
        "collecting_state",
        property(lambda self: collecting_state),
    )

    def refresh():
        state.is_inference_service_running = None

    snapshot = (
        None
        if status is None
        else {
            "data": status,
            "destination": "/episode",
            "waiting_topics": ["/camera"],
            "failure_reason": "disk full",
        }
    )
    component.ros_helper = types.SimpleNamespace(
        refresh_runtime_state=refresh,
        status_snapshot=lambda key: snapshot,
    )
    indicators, rendered, captions, errors, panels = [], [], [], [], []
    current_column = [None]

    @contextmanager
    def column(index):
        current_column[0] = index
        yield
        current_column[0] = None

    def columns(count):
        assert count == 3
        return [column(index) for index in range(count)]

    def expander(label, **kwargs):
        panels.append(label)
        return _Expander()

    def indicator(**kwargs):
        rendered.append((current_column[0], "indicator"))
        indicators.append(kwargs)

    monkeypatch.setattr(control.st, "expander", expander, raising=False)
    monkeypatch.setattr(control.st, "columns", columns, raising=False)
    monkeypatch.setattr(control.st, "caption", captions.append, raising=False)
    monkeypatch.setattr(control.st, "error", errors.append, raising=False)
    monkeypatch.setattr(
        control.st,
        "markdown",
        lambda title: rendered.append((current_column[0], title)),
        raising=False,
    )
    monkeypatch.setattr(
        control,
        "multi_status_indicator",
        indicator,
    )

    component._render_state_panel()

    assert panels == ["ℹ️ Current State"]
    assert rendered == [
        (0, "**Control**"),
        (0, "indicator"),
        (1, "**Recording**"),
        (1, "indicator"),
        (2, "**Inference**"),
        (2, "indicator"),
    ]
    assert indicators[0]["current_status"] == "takeover"
    assert indicators[0]["status_config"]["resetting"].text == "Resetting"
    assert indicators[0]["status_config"]["resetting"].color == "orange"
    assert indicators[1]["current_status"] == status
    if status is not None:
        assert indicators[1]["status_config"][status].text == (
            status.capitalize()
        )
        assert indicators[1]["status_config"][status].color == color
    else:
        assert None not in indicators[1]["status_config"]
    assert indicators[2]["current_status"] is None
    assert indicators[2]["status_config"][True].text == "Enabled"
    assert indicators[2]["status_config"][False].text == "Disabled"
    assert ("/episode" in captions) is (status is not None)
    assert ("Waiting for: /camera" in captions) is (status == "waiting")
    assert ("Start requested; waiting for Recorder status." in captions) is (
        pending == "start"
    )
    assert (
        "Waiting for Recorder completion or metadata settlement." in captions
    ) is (pending == "stop")
    assert errors == (["disk full"] if status == "failed" else [])
