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

import importlib.util
import sys
import types
from pathlib import Path

from robo_orchard_inference_app.config import TaskCfg


class _DummyLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _load_task_config_module():
    package_root = (
        Path(__file__).resolve().parents[3] / "robo_orchard_inference_app"
    )
    module_path = package_root / "components" / "task_config.py"

    streamlit_stub = types.SimpleNamespace(
        session_state=types.SimpleNamespace(
            collecting_state=types.SimpleNamespace(is_recording=False),
        ),
        write=lambda *args, **kwargs: None,
        rerun=lambda: None,
    )
    streamlit_tags_stub = types.ModuleType("streamlit_tags")
    streamlit_tags_stub.st_tags = lambda *args, **kwargs: []

    class _ComponentBase:
        def __init__(self, key_prefix=None):
            self.key_prefix = (
                key_prefix
                if key_prefix is not None
                else self.__class__.__name__
            )

        @property
        def task_cfg(self):
            return streamlit_stub.session_state.collecting_state.task_cfg

        @property
        def collecting_state(self):
            return streamlit_stub.session_state.collecting_state

        @property
        def logger(self):
            return _DummyLogger()

    mixin_stub = types.SimpleNamespace(ComponentBase=_ComponentBase)

    spec = importlib.util.spec_from_file_location(
        "task_config_module", module_path
    )
    module = importlib.util.module_from_spec(spec)
    module.__dict__["st"] = streamlit_stub
    sys.modules["streamlit"] = streamlit_stub
    sys.modules["streamlit_tags"] = streamlit_tags_stub
    sys.modules[
        "robo_orchard_inference_app.components.mixin"
    ] = mixin_stub
    sys.modules["robo_orchard_inference_app.utils"] = types.SimpleNamespace(
        time_str_now=lambda: "20260101_000000"
    )
    spec.loader.exec_module(module)
    return module, streamlit_stub


def test_render_tf_directories_is_locked_during_recording():
    module, st_stub = _load_task_config_module()
    task_cfg = TaskCfg(candidate_tf_directories=["/media/tf_v1"])
    written = []

    collecting_state = types.SimpleNamespace(
        task_cfg=task_cfg,
        is_recording=True,
    )
    st_stub.session_state.collecting_state = collecting_state
    st_stub.write = lambda msg: written.append(msg)

    component = module.TaskConfigComponent.__new__(
        module.TaskConfigComponent
    )
    component.key_prefix = "test"
    component._dump_task_cfg_to_disk = lambda: None

    component._render_tf_directories()

    assert written != []
    assert task_cfg.candidate_tf_directories == ["/media/tf_v1"]


def test_render_tf_directories_updates_on_change():
    module, st_stub = _load_task_config_module()
    task_cfg = TaskCfg(candidate_tf_directories=["/media/tf_v1"])
    written = []
    rerun_called = {"count": 0}

    collecting_state = types.SimpleNamespace(
        task_cfg=task_cfg,
        is_recording=False,
    )
    st_stub.session_state.collecting_state = collecting_state
    st_stub.rerun = lambda: rerun_called.__setitem__(
        "count", rerun_called["count"] + 1
    )

    # Patch the module-level st_tags binding
    # (imported via `from streamlit_tags import st_tags`)
    module.st_tags = lambda *args, **kwargs: ["/media/tf_v1", "/media/tf_v2"]

    component = module.TaskConfigComponent.__new__(
        module.TaskConfigComponent
    )
    component.key_prefix = "test"
    component._dump_task_cfg_to_disk = lambda: written.append(True)

    component._render_tf_directories()

    assert task_cfg.candidate_tf_directories == [
        "/media/tf_v1",
        "/media/tf_v2",
    ]
    assert written == [True]
    assert rerun_called["count"] == 1
