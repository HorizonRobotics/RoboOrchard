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


def test_reset_is_disabled_in_stop_mode():
    component = _build_component(
        is_inference_service_running=False,
        control_mode="stop",
    )

    assert component._is_reset_disabled() is True
