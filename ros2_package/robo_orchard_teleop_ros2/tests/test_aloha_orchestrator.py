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

import sys
import types
from pathlib import Path

import pytest


def _install_stub_modules():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda *args, **kwargs: None
    fake_rclpy.shutdown = lambda *args, **kwargs: None

    fake_node_module = types.ModuleType("rclpy.node")
    fake_callback_groups = types.ModuleType("rclpy.callback_groups")
    fake_executors = types.ModuleType("rclpy.executors")

    class ParameterDescriptor:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class MutuallyExclusiveCallbackGroup:
        pass

    class ReentrantCallbackGroup:
        pass

    class MultiThreadedExecutor:
        def __init__(self, *args, **kwargs):
            self.nodes = []

        def add_node(self, node):
            self.nodes.append(node)

        def spin(self):
            pass

    class FakeFuture:
        def __init__(self, response):
            self._response = response

        def add_done_callback(self, callback):
            callback(self)

        def result(self):
            return self._response

    class FakeClient:
        def __init__(self, service_name, response, call_order, ready=True):
            self.service_name = service_name
            self.response = response
            self.call_order = call_order
            self.ready = ready
            self.calls = 0

        def wait_for_service(self, timeout_sec=None):
            return self.ready

        def remove_pending_request(self, future):
            pass

        def call_async(self, request):
            self.calls += 1
            self.call_order.append(self.service_name)
            return FakeFuture(self.response)

    class FakeNode:
        parameter_overrides = {}
        service_results = {}
        unavailable_services = set()
        last_instance = None

        def __init__(self, *args, **kwargs):
            self._declared_parameters = {}
            self._created_clients = []
            self._created_services = []
            self._client_call_order = []
            self._logs = {"info": [], "warn": [], "error": []}
            FakeNode.last_instance = self

        def declare_parameter(self, name, value, descriptor=None):
            self._declared_parameters[name] = FakeNode.parameter_overrides.get(
                name, value
            )

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._declared_parameters[name])

        def create_client(
            self, service_type, service_name, callback_group=None
        ):
            response = FakeNode.service_results.get(
                service_name, _trigger_response(True, f"{service_name} ok")
            )
            client = FakeClient(
                service_name=service_name,
                response=response,
                call_order=self._client_call_order,
                ready=service_name not in FakeNode.unavailable_services,
            )
            self._created_clients.append(client)
            return client

        def create_service(
            self, service_type, name, callback, callback_group=None
        ):
            self._created_services.append(name)
            return types.SimpleNamespace(name=name, callback=callback)

        def get_logger(self):
            logs = self._logs
            return types.SimpleNamespace(
                info=lambda message: logs["info"].append(message),
                warn=lambda message: logs["warn"].append(message),
                error=lambda message: logs["error"].append(message),
            )

        def destroy_node(self):
            pass

    fake_node_module.Node = FakeNode
    fake_node_module.ParameterDescriptor = ParameterDescriptor
    fake_callback_groups.MutuallyExclusiveCallbackGroup = (
        MutuallyExclusiveCallbackGroup
    )
    fake_callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup
    fake_executors.MultiThreadedExecutor = MultiThreadedExecutor

    fake_rclpy.node = fake_node_module
    fake_rclpy.callback_groups = fake_callback_groups
    fake_rclpy.executors = fake_executors

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")

    class Trigger:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.success = None
                self.message = ""

    std_srvs.srv = std_srvs_srv
    std_srvs_srv.Trigger = Trigger

    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_node_module
    sys.modules["rclpy.callback_groups"] = fake_callback_groups
    sys.modules["rclpy.executors"] = fake_executors
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv

    return FakeNode


def _trigger_response(success, message):
    response = Trigger.Response()
    response.success = success
    response.message = message
    return response


FakeNode = _install_stub_modules()

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1]),
)

from std_srvs.srv import Trigger  # noqa: E402

from robo_orchard_teleop_ros2.take_over.orchestrator.aloha import (  # noqa: E402
    AlohaOrchestratorNode,
)


def _build_node(
    enable_services=None,
    muxer_release_service="/robot/left/takeover_muxer/release_control",
    muxer_takeover_service="/robot/left/takeover_muxer/trigger_takeover",
    service_results=None,
    unavailable_services=None,
):
    if enable_services is None:
        enable_services = [
            "/robot/left/enable_ctrl",
            "/robot/left_master/enable_ctrl",
        ]

    FakeNode.parameter_overrides = {
        "enable_services": enable_services,
        "muxer_release_service": muxer_release_service,
        "muxer_takeover_service": muxer_takeover_service,
    }
    FakeNode.service_results = service_results or {}
    FakeNode.unavailable_services = set(unavailable_services or [])

    try:
        return AlohaOrchestratorNode()
    finally:
        FakeNode.parameter_overrides = {}
        FakeNode.service_results = {}
        FakeNode.unavailable_services = set()


def _call_auto(node):
    response = Trigger.Response()
    ret = node._auto_service_callback(Trigger.Request(), response)
    assert ret is response
    return response


def _call_takeover(node):
    response = Trigger.Response()
    ret = node._takeover_service_callback(Trigger.Request(), response)
    assert ret is response
    return response


def _client_by_service(node, service_name):
    return next(
        client
        for client in node._created_clients
        if client.service_name == service_name
    )


def test_auto_enables_all_services_then_muxer():
    enable_services = ["/arm/a/enable_ctrl", "/arm/b/enable_ctrl"]
    release_service = "/muxer/release_control"
    node = _build_node(
        enable_services=enable_services,
        muxer_release_service=release_service,
    )

    response = _call_auto(node)

    assert response.success is True
    assert response.message == "Auto mode activated."
    assert node._client_call_order == [
        "/arm/a/enable_ctrl",
        "/arm/b/enable_ctrl",
        "/muxer/release_control",
    ]


def test_auto_enable_failure_short_circuits():
    enable_services = [
        "/arm/a/enable_ctrl",
        "/arm/b/enable_ctrl",
        "/arm/c/enable_ctrl",
    ]
    release_service = "/muxer/release_control"
    node = _build_node(
        enable_services=enable_services,
        muxer_release_service=release_service,
        service_results={
            "/arm/b/enable_ctrl": _trigger_response(
                False, "arm b still in teach mode"
            )
        },
    )

    response = _call_auto(node)

    assert response.success is False
    assert "/arm/b/enable_ctrl" in response.message
    assert "arm b still in teach mode" in response.message
    assert node._client_call_order == [
        "/arm/a/enable_ctrl",
        "/arm/b/enable_ctrl",
    ]
    assert _client_by_service(node, "/arm/c/enable_ctrl").calls == 0
    assert _client_by_service(node, release_service).calls == 0


def test_auto_muxer_release_fails_returns_failure():
    release_service = "/muxer/release_control"
    node = _build_node(
        enable_services=["/arm/a/enable_ctrl", "/arm/b/enable_ctrl"],
        muxer_release_service=release_service,
        service_results={
            release_service: _trigger_response(False, "muxer refused release")
        },
    )

    response = _call_auto(node)

    assert response.success is False
    assert response.message == "muxer refused release"
    assert node._client_call_order == [
        "/arm/a/enable_ctrl",
        "/arm/b/enable_ctrl",
        release_service,
    ]


def test_takeover_emits_reminder_and_calls_muxer():
    takeover_service = "/muxer/trigger_takeover"
    node = _build_node(muxer_takeover_service=takeover_service)

    response = _call_takeover(node)

    assert response.success is True
    assert response.message == "Takeover mode activated."
    assert node._client_call_order == [takeover_service]
    assert any("hardware teach button" in log for log in node._logs["info"])


def test_takeover_muxer_trigger_fails_returns_failure():
    takeover_service = "/muxer/trigger_takeover"
    node = _build_node(
        muxer_takeover_service=takeover_service,
        service_results={
            takeover_service: _trigger_response(
                False, "muxer refused takeover"
            )
        },
    )

    response = _call_takeover(node)

    assert response.success is False
    assert response.message == "muxer refused takeover"
    assert node._client_call_order == [takeover_service]


def test_takeover_does_not_call_any_enable():
    enable_services = ["/arm/a/enable_ctrl", "/arm/b/enable_ctrl"]
    node = _build_node(enable_services=enable_services)

    _call_takeover(node)

    for service_name in enable_services:
        assert _client_by_service(node, service_name).calls == 0


@pytest.mark.parametrize(
    "enable_services, muxer_release_service, "
    "muxer_takeover_service, param_name",
    [
        (
            [],
            "/muxer/release_control",
            "/muxer/trigger_takeover",
            "enable_services",
        ),
        (
            [""],
            "/muxer/release_control",
            "/muxer/trigger_takeover",
            "enable_services",
        ),
        (
            ["/arm/enable_ctrl"],
            "",
            "/muxer/trigger_takeover",
            "muxer_release_service",
        ),
        (
            ["/arm/enable_ctrl"],
            "/muxer/release_control",
            "",
            "muxer_takeover_service",
        ),
    ],
)
def test_empty_parameter_fails_construction(
    enable_services,
    muxer_release_service,
    muxer_takeover_service,
    param_name,
):
    FakeNode.last_instance = None

    with pytest.raises(ValueError):
        _build_node(
            enable_services=enable_services,
            muxer_release_service=muxer_release_service,
            muxer_takeover_service=muxer_takeover_service,
        )

    assert FakeNode.last_instance is not None
    assert any(
        param_name in message
        for message in FakeNode.last_instance._logs["error"]
    )
