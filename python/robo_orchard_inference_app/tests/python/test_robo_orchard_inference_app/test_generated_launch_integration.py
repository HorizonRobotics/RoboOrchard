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

import importlib.util
import io
import pathlib
import sys
import types

import pytest


def _install_stub_modules():
    version = types.ModuleType("robo_orchard_inference_app.version")
    version.__version__ = "0.0.0"
    version.__full_version__ = "0.0.0"
    version.__git_hash__ = "test"
    sys.modules["robo_orchard_inference_app.version"] = version

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
    sys.modules["streamlit.components"] = st_components
    sys.modules["streamlit.components.v1"] = st_components_v1

    polling2 = types.ModuleType("polling2")
    polling2.TimeoutException = RuntimeError
    polling2.poll = lambda *args, **kwargs: None
    sys.modules["polling2"] = polling2

    roslibpy = types.ModuleType("roslibpy")

    class FakeTimeoutError(Exception):
        pass

    class FakeService:
        def __init__(self, client, name, service_type):
            self.client = client
            self.name = name
            self.service_type = service_type

        def call(self, request, timeout=5.0):
            self.client.called_services.append(self.name)
            return self.client.service_results[self.name]

    class FakeServiceRequest(dict):
        def __init__(self, data=None):
            super().__init__(data or {})

    roslibpy.Service = FakeService
    roslibpy.ServiceRequest = FakeServiceRequest
    roslibpy.Topic = object
    roslibpy.core = types.SimpleNamespace(RosTimeoutError=FakeTimeoutError)
    roslibpy.Ros = object
    sys.modules["roslibpy"] = roslibpy

    streamlit_tags = types.ModuleType("streamlit_tags")
    streamlit_tags.st_tags = lambda *args, **kwargs: []
    sys.modules["streamlit_tags"] = streamlit_tags


_install_stub_modules()
sys.path.insert(0, "python/robo_orchard_inference_app")

from robo_orchard_inference_app.components.main_control import (  # noqa: E402
    MainControlComponent,
)
from robo_orchard_inference_app.config import LaunchCfg  # noqa: E402
from robo_orchard_inference_app.ros_bridge import RosServiceHelper  # noqa: E402
from robo_orchard_inference_app.state import (  # noqa: E402
    CollectingState,
    InferenceState,
)


class FakeRosClient:
    def __init__(self, services):
        self.is_connected = True
        self.service_results = {
            service: {"success": True, "message": "ok"} for service in services
        }
        self.called_services = []

    def get_services(self):
        return list(self.service_results.keys())

    def get_nodes(self):
        return []


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.errors = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)

    def warning(self, message):
        self.warnings.append(message)


def _find_repo_file(relative_path: str) -> pathlib.Path:
    current = pathlib.Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / relative_path
        if candidate.exists():
            return candidate
    raise FileNotFoundError(relative_path)


def _load_generated_launch_cfg(monkeypatch) -> LaunchCfg:
    module_path = _find_repo_file(
        "projects/HoloBrain/app/gen_inference_app_launch_config.py"
    )
    spec = importlib.util.spec_from_file_location(
        "generated_launch_cfg_module", module_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    written = {}

    class _Capture(io.StringIO):
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            written["content"] = self.getvalue()

    monkeypatch.setattr("builtins.open", lambda *args, **kwargs: _Capture())
    module.main()
    return LaunchCfg.model_validate_json(written["content"])


def test_generated_launch_cfg_uses_set_static_transforms_service(
    monkeypatch,
):
    launch_cfg = _load_generated_launch_cfg(monkeypatch)

    assert launch_cfg.ros_bridge.static_transform_service_name == (
        "/set_static_transforms"
    )


def _build_helper_from_generated_cfg(monkeypatch):
    _install_stub_modules()
    import robo_orchard_inference_app.ros_bridge as ros_bridge_module

    monkeypatch.setattr(
        ros_bridge_module,
        "roslibpy",
        sys.modules["roslibpy"],
    )

    launch_cfg = _load_generated_launch_cfg(monkeypatch)
    cfg = launch_cfg.ros_bridge
    all_services = (
        cfg.takeover_service_name
        + cfg.auto_service_name
        + cfg.stop_service_name
        + cfg.disable_inference_service_name
        + cfg.reset_service_name
    )
    logger = FakeLogger()
    helper = RosServiceHelper(
        ros_client=FakeRosClient(all_services),
        ros_bridge_cfg=cfg,
        inference_state=InferenceState(),
        logger=logger,
    )
    return launch_cfg, helper, logger


def test_generated_launch_cfg_uses_global_manager_services(monkeypatch):
    _, helper, logger = _build_helper_from_generated_cfg(monkeypatch)
    cfg = helper.cfg
    assert cfg.takeover_service_name == ["/robot/control/takeover"]
    assert cfg.auto_service_name == ["/robot/control/auto"]
    assert cfg.stop_service_name == ["/robot/control/stop"]
    assert cfg.reset_service_name == ["/robot/control/reset"]
    assert cfg.enable_inference_service_name == [
        "/robot/inference_service/enable"
    ]
    assert cfg.disable_inference_service_name == [
        "/robot/inference_service/disable"
    ]


def test_app_reset_calls_only_global_manager_reset(monkeypatch):
    _, helper, logger = _build_helper_from_generated_cfg(monkeypatch)

    reset_calls = []

    def fake_reset_arm():
        reset_calls.append("reset_arm")
        return True

    monkeypatch.setattr(helper, "reset_arm", fake_reset_arm)

    component = MainControlComponent.__new__(MainControlComponent)
    component.ros_helper = helper
    collecting_state = CollectingState(
        inference_state=InferenceState(
            control_mode="auto",
            is_inference_service_running=True,
        )
    )
    monkeypatch.setattr(
        MainControlComponent,
        "collecting_state",
        property(lambda _self: collecting_state),
        raising=False,
    )
    monkeypatch.setattr(
        MainControlComponent,
        "logger",
        property(lambda _self: logger),
        raising=False,
    )

    component.reset_arm_ctrl_callback()

    assert reset_calls == ["reset_arm"]
    assert logger.warnings == []


@pytest.mark.parametrize(
    ("mode", "service"),
    [
        ("auto", "/robot/control/auto"),
        ("takeover", "/robot/control/takeover"),
        ("stop", "/robot/control/stop"),
    ],
)
def test_control_modes_call_only_global_manager(monkeypatch, mode, service):
    _, helper, _logger = _build_helper_from_generated_cfg(monkeypatch)

    assert helper.set_control_mode(mode) is True

    assert helper.ros_client.called_services == [service]
    assert helper.state.control_mode is None
