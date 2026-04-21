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
