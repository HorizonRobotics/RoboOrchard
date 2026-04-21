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
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

# ---------------------------------------------------------------------------
# Stub heavy dependencies BEFORE any robo_orchard_inference_app import so
# that the launch_testing hook (which imports this file at collection time)
# does not choke.  The streamlit stub must be complete enough that files
# collected later (e.g. test_generated_launch_integration.py,
# test_main_control_node_status.py) can rely on it when their own
# sys.modules.setdefault("streamlit", …) calls become no-ops.
# ---------------------------------------------------------------------------


class _CtxNoop:
    def __enter__(self):
        return None

    def __exit__(self, *_):
        return False


_st_stub = types.ModuleType("streamlit")
_st_stub.session_state = types.SimpleNamespace()
_st_stub.toast = lambda *args, **kwargs: None
_st_stub.cache_resource = lambda func: func
_st_stub.rerun = lambda: None
_st_stub.dialog = lambda *args, **kwargs: (lambda func: func)
_st_stub.set_page_config = lambda *args, **kwargs: None
_st_stub.expander = lambda *args, **kwargs: _CtxNoop()
sys.modules.setdefault("streamlit", _st_stub)

_st_components = types.ModuleType("streamlit.components")
_st_components_v1 = types.ModuleType("streamlit.components.v1")
_st_components_v1.iframe = lambda *args, **kwargs: None
sys.modules.setdefault("streamlit.components", _st_components)
sys.modules.setdefault("streamlit.components.v1", _st_components_v1)

_roslibpy_stub = types.ModuleType("roslibpy")
_roslibpy_stub.Ros = object
_roslibpy_stub.Service = object
_roslibpy_stub.ServiceRequest = dict
_roslibpy_stub.Topic = object
_roslibpy_stub.core = types.SimpleNamespace(RosTimeoutError=RuntimeError)
sys.modules.setdefault("roslibpy", _roslibpy_stub)

_psutil_stub = types.ModuleType("psutil")
sys.modules.setdefault("psutil", _psutil_stub)

_polling2_stub = types.ModuleType("polling2")
_polling2_stub.TimeoutException = RuntimeError
_polling2_stub.poll = lambda *args, **kwargs: None
sys.modules.setdefault("polling2", _polling2_stub)

_streamlit_tags_stub = types.ModuleType("streamlit_tags")
_streamlit_tags_stub.st_tags = lambda *args, **kwargs: []
sys.modules.setdefault("streamlit_tags", _streamlit_tags_stub)

_version_stub = types.ModuleType("robo_orchard_inference_app.version")
_version_stub.__version__ = "0.0.0"
_version_stub.__full_version__ = "0.0.0"
_version_stub.__git_hash__ = "test"
sys.modules.setdefault("robo_orchard_inference_app.version", _version_stub)

from robo_orchard_inference_app.config import TaskCfg
from robo_orchard_inference_app.state import EpisodeMeta


class _DummyLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(msg)

    def info(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _make_component(task_cfg, episode_meta):
    """Return a bare EditEpisodeMetaComponent with minimal stubs.

    Returns (component, rerun_called, dummy_logger, eem_st) where eem_st is
    the actual `st` object bound inside edit_episode_meta — that is the
    object callers must patch, NOT the result of `import streamlit`.
    """
    import robo_orchard_inference_app.components.edit_episode_meta as _eem

    from robo_orchard_inference_app.components.edit_episode_meta import (
        EditEpisodeMetaComponent,
    )

    eem_st = _eem.st  # the st module that edit_episode_meta.py uses

    dummy_logger = _DummyLogger()
    # logger is a property returning st.session_state.logger
    eem_st.session_state.logger = dummy_logger

    component = object.__new__(EditEpisodeMetaComponent)
    component.episode_meta = episode_meta
    component.key_prefix = "test"
    rerun_called = {"count": 0}
    component.rerun_callback = (
        lambda: rerun_called.__setitem__(
            "count", rerun_called["count"] + 1
        )
    )
    component._task_cfg = task_cfg
    component._collecting_state = types.SimpleNamespace(is_recording=False)

    # stub task_cfg property
    type(component).task_cfg = property(lambda self: self._task_cfg)
    type(component).collecting_state = property(
        lambda self: self._collecting_state
    )

    return component, rerun_called, dummy_logger, eem_st


def test_render_tf_directory_no_candidates_shows_message(monkeypatch):
    task_cfg = TaskCfg(candidate_tf_directories=[])
    episode_meta = EpisodeMeta(tf_directory="")
    component, rerun_called, _, eem_st = _make_component(
        task_cfg, episode_meta
    )

    written = []
    monkeypatch.setattr(
        eem_st, "columns", lambda spec: [_CtxNoop(), _CtxNoop()], raising=False
    )
    monkeypatch.setattr(
        eem_st, "write", lambda msg: written.append(msg), raising=False
    )
    monkeypatch.setattr(
        eem_st, "selectbox", lambda *a, **kw: None, raising=False
    )

    component._render_tf_directory()

    assert any("No TF directories" in str(w) for w in written)
    assert rerun_called["count"] == 0


def test_render_tf_directory_no_candidates_clears_stale_selection(monkeypatch):
    task_cfg = TaskCfg(candidate_tf_directories=[])
    episode_meta = EpisodeMeta(tf_directory="/media/tf_old")
    component, _, dummy_logger, eem_st = _make_component(
        task_cfg, episode_meta
    )

    monkeypatch.setattr(
        eem_st, "columns", lambda spec: [_CtxNoop(), _CtxNoop()], raising=False
    )
    monkeypatch.setattr(
        eem_st, "write", lambda *a, **kw: None, raising=False
    )
    monkeypatch.setattr(
        eem_st, "selectbox", lambda *a, **kw: None, raising=False
    )

    component._render_tf_directory()

    assert episode_meta.tf_directory == ""
    assert len(dummy_logger.warnings) == 1


def test_render_tf_directory_resets_stale_selection(monkeypatch):
    task_cfg = TaskCfg(candidate_tf_directories=["/media/tf_v2"])
    episode_meta = EpisodeMeta(tf_directory="/media/tf_old")
    component, rerun_called, dummy_logger, eem_st = _make_component(
        task_cfg, episode_meta
    )

    monkeypatch.setattr(
        eem_st, "columns", lambda spec: [_CtxNoop(), _CtxNoop()], raising=False
    )
    monkeypatch.setattr(
        eem_st, "write", lambda *a, **kw: None, raising=False
    )
    monkeypatch.setattr(
        eem_st, "selectbox", lambda *a, **kw: None, raising=False
    )

    component._render_tf_directory()

    assert episode_meta.tf_directory == ""
    assert len(dummy_logger.warnings) == 1


def test_render_tf_directory_triggers_rerun_on_selection_change(monkeypatch):
    task_cfg = TaskCfg(
        candidate_tf_directories=["/media/tf_v1", "/media/tf_v2"]
    )
    episode_meta = EpisodeMeta(tf_directory="/media/tf_v1")
    component, rerun_called, _, eem_st = _make_component(
        task_cfg, episode_meta
    )

    monkeypatch.setattr(
        eem_st, "columns", lambda spec: [_CtxNoop(), _CtxNoop()], raising=False
    )
    monkeypatch.setattr(
        eem_st, "write", lambda *a, **kw: None, raising=False
    )
    monkeypatch.setattr(
        eem_st, "selectbox", lambda *a, **kw: "/media/tf_v2", raising=False
    )

    component._render_tf_directory()

    assert episode_meta.tf_directory == "/media/tf_v2"
    assert rerun_called["count"] == 1


def test_render_tf_directory_locked_while_recording(monkeypatch):
    task_cfg = TaskCfg(
        candidate_tf_directories=["/media/tf_v1", "/media/tf_v2"]
    )
    episode_meta = EpisodeMeta(tf_directory="/media/tf_v1")
    component, rerun_called, _, eem_st = _make_component(
        task_cfg, episode_meta
    )
    component._collecting_state.is_recording = True

    written = []
    selectbox_calls = []
    monkeypatch.setattr(
        eem_st, "columns", lambda spec: [_CtxNoop(), _CtxNoop()], raising=False
    )
    monkeypatch.setattr(
        eem_st, "write", lambda msg: written.append(msg), raising=False
    )
    monkeypatch.setattr(
        eem_st,
        "selectbox",
        lambda *a, **kw: selectbox_calls.append((a, kw)) or "/media/tf_v2",
        raising=False,
    )

    component._render_tf_directory()

    assert selectbox_calls == []
    assert episode_meta.tf_directory == "/media/tf_v1"
    assert rerun_called["count"] == 0
    assert any("locked while recording" in str(w) for w in written)
    assert any("/media/tf_v1" in str(w) for w in written)
