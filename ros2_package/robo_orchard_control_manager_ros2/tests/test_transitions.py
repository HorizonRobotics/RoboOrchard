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

import threading
import time

import pytest
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger

from robo_orchard_control_manager_ros2.node import (
    AUTO_SERVICE,
    EVENTS_TOPIC,
    RESET_SERVICE,
    STATUS_TOPIC,
    STOP_SERVICE,
    TAKEOVER_SERVICE,
    ControlManagerNode,
    ControlState,
)


def _make_node(
    tmp_path,
    *,
    enable_services=None,
    inference_disable_services=None,
    inference_node_candidates=None,
    reset_services=None,
    response_timeout=0.2,
):
    Node.reset_service_behaviors()
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "channels": [
                    {
                        "name": "arm",
                        "kind": "joint_command",
                        "msg_type": "sensor_msgs/msg/JointState",
                        "autonomous_topic": "/algo_cmd",
                        "override_topic": "/override_cmd",
                        "output_topic": "/robot/joint_cmd",
                    }
                ],
                "enable_services": enable_services or [],
                "inference_disable_services": (
                    inference_disable_services or []
                ),
                "inference_node_candidates": inference_node_candidates or [],
                "reset_services": reset_services or [],
                "replay_time_s": 0.0,
                "service_wait_timeout_s": 0.1,
                "service_response_timeout_s": response_timeout,
            }
        ),
        encoding="utf-8",
    )
    Node.config_file = str(config_path)
    return ControlManagerNode()


def _call(node, service_name):
    response = Trigger.Response()
    return node.services[service_name].callback(Trigger.Request(), response)


def _make_optional_inference_node(tmp_path):
    return _make_node(
        tmp_path,
        inference_disable_services=["/disable"],
        inference_node_candidates=[
            "/robot/inference/sync_node",
            "/robot/inference/async_node",
        ],
        reset_services=["/reset"],
        response_timeout=0.01,
    )


@pytest.mark.parametrize("reset_success", [True, False])
def test_reset_without_deploy_skips_disable_without_waiting(
    tmp_path, monkeypatch, reset_success
):
    node = _make_optional_inference_node(tmp_path)
    node.config.service_wait_timeout_s = 5.0
    node.graph_nodes = [("sync_node", "/unrelated")]
    Node.configure_service("/disable", available=False)
    Node.configure_service("/reset", success=reset_success)

    def unexpected_sleep(duration):
        pytest.fail("Reset must not wait for an absent optional Deploy")

    monkeypatch.setattr(time, "sleep", unexpected_sleep)
    response = _call(node, RESET_SERVICE)

    assert response.success is reset_success
    assert node.service_call_order == ["/reset"]
    assert node.clients["/disable"].wait_timeouts == []
    assert any(
        "skipping inference disable" in text
        for text in node.logger.info_messages
    )
    assert node.state == ControlState.STOP
    assert [
        message.data
        for message in node.publishers[STATUS_TOPIC].published[-2:]
    ] == ["resetting", "stop"]


def test_reset_without_candidates_keeps_disable_mandatory(tmp_path):
    node = _make_node(
        tmp_path,
        inference_disable_services=["/disable"],
        reset_services=["/reset"],
    )
    Node.configure_service("/disable", available=False)

    response = _call(node, RESET_SERVICE)

    assert not response.success
    assert "discovery timeout" in response.message
    assert node.service_call_order == []
    assert node.state == ControlState.STOP


@pytest.mark.parametrize("node_name", ["sync_node", "async_node"])
@pytest.mark.parametrize(
    "message", ["Node paused.", "Node is already paused."]
)
def test_discovered_deploy_requires_idempotent_disable(
    tmp_path, node_name, message
):
    node = _make_optional_inference_node(tmp_path)
    node.graph_nodes = [(node_name, "/robot/inference")]
    Node.configure_service("/disable", success=True, message=message)

    response = _call(node, RESET_SERVICE)

    assert response.success
    assert node.service_call_order == ["/disable", "/reset"]
    assert node.state == ControlState.STOP


@pytest.mark.parametrize("failure", ["unavailable", "rejected", "timeout"])
def test_present_deploy_disable_failure_blocks_hardware_reset(
    tmp_path, failure
):
    node = _make_optional_inference_node(tmp_path)
    node.graph_nodes = [("sync_node", "/robot/inference")]
    Node.configure_service(
        "/disable",
        available=failure != "unavailable",
        success=failure != "rejected",
        pending=failure == "timeout",
    )

    response = _call(node, RESET_SERVICE)

    assert not response.success
    assert "hardware reset skipped" in response.message
    assert "/reset" not in node.service_call_order
    assert node.state == ControlState.STOP


@pytest.mark.parametrize("success", [True, False])
def test_ready_disable_service_is_not_skipped_for_unmatched_node(
    tmp_path, success
):
    node = _make_optional_inference_node(tmp_path)
    node.graph_nodes = [("renamed_deploy", "/custom")]
    Node.configure_service("/disable", success=success)

    response = _call(node, RESET_SERVICE)

    assert response.success is success
    assert node.service_call_order == (
        ["/disable", "/reset"] if success else ["/disable"]
    )


@pytest.mark.parametrize("endpoint", ["node", "service"])
def test_deploy_discovered_before_next_reset_is_not_skipped(
    tmp_path, endpoint
):
    node = _make_optional_inference_node(tmp_path)
    Node.configure_service("/disable", available=False)

    assert _call(node, RESET_SERVICE).success
    assert node.service_call_order == ["/reset"]
    node.service_call_order.clear()
    node.logger.info_messages.clear()
    if endpoint == "node":
        node.graph_nodes = [("async_node", "/robot/inference")]
    else:
        Node.configure_service("/disable", available=True)

    response = _call(node, RESET_SERVICE)

    assert response.success is (endpoint == "service")
    assert node.service_call_order == (
        ["/disable", "/reset"] if endpoint == "service" else []
    )
    assert not any("skipping" in text for text in node.logger.info_messages)


def test_disappearing_deploy_does_not_turn_disable_failure_into_skip(
    tmp_path, monkeypatch
):
    node = _make_optional_inference_node(tmp_path)
    node.graph_nodes = [("sync_node", "/robot/inference")]

    def unavailable(timeout_sec):
        node.graph_nodes = []
        return False

    monkeypatch.setattr(
        node.clients["/disable"], "wait_for_service", unavailable
    )
    response = _call(node, RESET_SERVICE)

    assert not response.success
    assert node.service_call_order == []
    assert node.state == ControlState.STOP


@pytest.mark.parametrize("endpoint", ["node", "service"])
def test_discovery_errors_do_not_skip_inference_disable(
    tmp_path, monkeypatch, endpoint
):
    node = _make_optional_inference_node(tmp_path)

    def fail():
        raise RuntimeError("discovery failed")

    if endpoint == "node":
        monkeypatch.setattr(node, "get_node_names_and_namespaces", fail)
    else:
        monkeypatch.setattr(node.clients["/disable"], "service_is_ready", fail)
    response = _call(node, RESET_SERVICE)

    assert not response.success
    assert "discovery failed" in response.message
    assert node.service_call_order == []
    assert node.state == ControlState.STOP


def test_one_discovered_service_keeps_all_disable_services_mandatory(tmp_path):
    node = _make_node(
        tmp_path,
        inference_disable_services=["/disable_a", "/disable_b"],
        inference_node_candidates=["/inference"],
        reset_services=["/reset"],
    )
    Node.configure_service("/disable_a", available=False)
    Node.configure_service("/disable_b", available=True)

    response = _call(node, RESET_SERVICE)

    assert not response.success
    assert node.service_call_order == ["/disable_b"]
    assert node.state == ControlState.STOP


def test_transition_endpoints_use_reentrant_callback_groups(tmp_path):
    node = _make_node(
        tmp_path,
        enable_services=["/enable"],
        inference_disable_services=["/disable"],
        reset_services=["/reset"],
    )

    assert set(node.services) == {
        AUTO_SERVICE,
        TAKEOVER_SERVICE,
        STOP_SERVICE,
        RESET_SERVICE,
    }
    assert all(
        service.callback_group is node._transition_callback_group
        for service in node.services.values()
    )
    assert all(
        subscription.callback_group is node._command_callback_group
        for subscription in node.subscriptions.values()
    )
    assert all(
        client.callback_group is node._downstream_callback_group
        for client in node.clients.values()
    )


def test_auto_enables_in_order_without_enabling_inference(tmp_path):
    node = _make_node(
        tmp_path,
        enable_services=["/enable_left", "/enable_right"],
        inference_disable_services=["/inference/disable"],
    )
    Node.configure_service("/enable_left", success=True)
    Node.configure_service("/enable_right", success=True)
    Node.configure_service("/inference/disable", success=True)

    response = _call(node, AUTO_SERVICE)

    assert response.success is True
    assert node.state == ControlState.AUTO
    assert node.service_call_order == ["/enable_left", "/enable_right"]
    assert node.publishers[STATUS_TOPIC].published[-1].data == "auto"


def test_auto_failure_keeps_gate_closed_and_stops_enable_sequence(tmp_path):
    node = _make_node(
        tmp_path,
        enable_services=["/enable_left", "/enable_right"],
    )
    Node.configure_service("/enable_left", success=False, message="fault")
    Node.configure_service("/enable_right", success=True)

    response = _call(node, AUTO_SERVICE)

    assert response.success is False
    assert "/enable_left: fault" in response.message
    assert node.service_call_order == ["/enable_left"]
    assert node.state == ControlState.STOP
    node.subscriptions["/algo_cmd"].callback(node.message_types["arm"]())
    assert node.publishers["/robot/joint_cmd"].published == []


def test_conflicting_transition_is_rejected_without_waiting(tmp_path):
    node = _make_node(tmp_path, enable_services=["/enable"])
    Node.configure_service("/enable", pending=True)
    node._change_state(ControlState.TAKEOVER)
    auto_response = Trigger.Response()

    auto_thread = threading.Thread(
        target=node.services[AUTO_SERVICE].callback,
        args=(Trigger.Request(), auto_response),
    )
    auto_thread.start()
    assert node.clients["/enable"].called.wait(timeout=2)
    assert node.state == ControlState.STOP

    stop_response = _call(node, STOP_SERVICE)

    assert stop_response.success is False
    assert "already in progress" in stop_response.message
    assert auto_thread.is_alive()

    node.clients["/enable"].futures[0].set_result(
        Trigger.Response(success=True, message="enabled")
    )
    auto_thread.join(timeout=2)
    assert not auto_thread.is_alive()
    assert auto_response.success is True
    assert node.state == ControlState.AUTO


def test_auto_takeover_and_stop_are_idempotent(tmp_path):
    node = _make_node(tmp_path, enable_services=["/enable"])
    Node.configure_service("/enable", success=True)

    assert _call(node, AUTO_SERVICE).success is True
    assert _call(node, AUTO_SERVICE).success is True
    assert node.service_call_order == ["/enable"]

    assert _call(node, TAKEOVER_SERVICE).success is True
    event_count = len(node.publishers[EVENTS_TOPIC].published)
    status_count = len(node.publishers[STATUS_TOPIC].published)
    assert _call(node, TAKEOVER_SERVICE).success is True
    assert len(node.publishers[EVENTS_TOPIC].published) == event_count
    assert len(node.publishers[STATUS_TOPIC].published) == status_count

    assert _call(node, STOP_SERVICE).success is True
    event_count = len(node.publishers[EVENTS_TOPIC].published)
    status_count = len(node.publishers[STATUS_TOPIC].published)
    assert _call(node, STOP_SERVICE).success is True
    assert len(node.publishers[EVENTS_TOPIC].published) == event_count
    assert len(node.publishers[STATUS_TOPIC].published) == status_count


def test_inference_disable_failure_skips_resets_and_allows_retry(tmp_path):
    node = _make_node(
        tmp_path,
        inference_disable_services=["/disable_a", "/disable_b"],
        reset_services=["/reset_a", "/reset_b"],
    )
    Node.configure_service("/disable_a", success=False, message="busy")
    Node.configure_service("/disable_b", success=True)
    Node.configure_service("/reset_a", success=True)
    Node.configure_service("/reset_b", success=True)

    first_response = _call(node, RESET_SERVICE)

    assert first_response.success is False
    assert "/disable_a: busy" in first_response.message
    assert node.service_call_order == ["/disable_a", "/disable_b"]
    assert node.state == ControlState.STOP

    Node.configure_service("/disable_a", success=True)
    second_response = _call(node, RESET_SERVICE)

    assert second_response.success is True
    assert node.state == ControlState.STOP
    assert "/reset_a" in node.service_call_order
    assert "/reset_b" in node.service_call_order
    assert node.publishers[EVENTS_TOPIC].published == []


def test_reset_suppresses_commands_and_starts_hardware_resets_together(
    tmp_path,
):
    node = _make_node(
        tmp_path,
        inference_disable_services=["/disable"],
        reset_services=["/reset_a", "/reset_b"],
    )
    Node.configure_service("/disable", success=True)
    Node.configure_service("/reset_a", pending=True)
    Node.configure_service("/reset_b", pending=True)
    reset_response = Trigger.Response()
    reset_thread = threading.Thread(
        target=node.services[RESET_SERVICE].callback,
        args=(Trigger.Request(), reset_response),
    )

    reset_thread.start()
    assert node.clients["/reset_a"].called.wait(timeout=2)
    assert node.clients["/reset_b"].called.wait(timeout=2)
    assert node.state == ControlState.RESETTING
    node.subscriptions["/algo_cmd"].callback(node.message_types["arm"]())
    node.subscriptions["/override_cmd"].callback(node.message_types["arm"]())
    assert node.publishers["/robot/joint_cmd"].published == []

    node.clients["/reset_a"].futures[0].set_result(
        Trigger.Response(success=False, message="left fault")
    )
    node.clients["/reset_b"].futures[0].set_result(
        Trigger.Response(success=False, message="right fault")
    )
    reset_thread.join(timeout=2)

    assert not reset_thread.is_alive()
    assert reset_response.success is False
    assert "/reset_a: left fault" in reset_response.message
    assert "/reset_b: right fault" in reset_response.message
    assert node.state == ControlState.STOP
    assert [
        message.data
        for message in node.publishers[STATUS_TOPIC].published[-2:]
    ] == ["resetting", "stop"]
    assert node.publishers[EVENTS_TOPIC].published == []


def test_service_discovery_and_response_timeouts_are_bounded(tmp_path):
    unavailable = _make_node(tmp_path, enable_services=["/enable"])
    Node.configure_service("/enable", available=False)

    unavailable_response = _call(unavailable, AUTO_SERVICE)

    assert unavailable_response.success is False
    assert "discovery timeout" in unavailable_response.message
    assert unavailable.clients["/enable"].wait_timeouts == [0.1]

    pending = _make_node(
        tmp_path, enable_services=["/enable"], response_timeout=0.01
    )
    Node.configure_service("/enable", pending=True)

    timeout_response = _call(pending, AUTO_SERVICE)

    assert timeout_response.success is False
    assert "response timed out" in timeout_response.message
    assert pending.state == ControlState.STOP
    client = pending.clients["/enable"]
    assert client.removed_requests == client.futures

    assert _call(pending, AUTO_SERVICE).success is False
    assert len(client.futures) == 2
    assert client.removed_requests == client.futures
