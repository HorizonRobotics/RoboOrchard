# Project RoboOrchard
#
# Copyright (c) 2025 Horizon Robotics. All Rights Reserved.
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

import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")
TransformStamped = pytest.importorskip("geometry_msgs.msg").TransformStamped
SingleThreadedExecutor = pytest.importorskip(
    "rclpy.executors"
).SingleThreadedExecutor
Node = pytest.importorskip("rclpy.node").Node
Buffer = pytest.importorskip("tf2_ros.buffer").Buffer
StaticTransformBroadcaster = pytest.importorskip(
    "tf2_ros.static_transform_broadcaster"
).StaticTransformBroadcaster
TransformListener = pytest.importorskip(
    "tf2_ros.transform_listener"
).TransformListener


def _spin_until(executor, predicate, timeout_sec: float):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        value = predicate()
        if value is not None:
            return value
    return None


def _lookup_translation_x(buffer: Buffer):
    try:
        transform = buffer.lookup_transform(
            "world", "camera", rclpy.time.Time()
        )
    except Exception:
        return None
    return transform.transform.translation.x


def _publish_static_transform(
    node: Node, broadcaster: StaticTransformBroadcaster, x: float
):
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = "world"
    transform.child_frame_id = "camera"
    transform.transform.translation.x = x
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform([transform])


@pytest.fixture
def ros_graph(monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp")
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []

    def create_node(prefix: str):
        node = Node(f"{prefix}_{uuid.uuid4().hex}")
        executor.add_node(node)
        nodes.append(node)
        return node

    try:
        yield executor, create_node
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def test_static_tf_republish_updates_existing_listener(ros_graph):
    executor, create_node = ros_graph
    publisher_node = create_node("tf_pub")
    listener_node = create_node("tf_listener")

    buffer = Buffer()
    TransformListener(buffer, listener_node, spin_thread=False)
    broadcaster = StaticTransformBroadcaster(publisher_node)

    _publish_static_transform(publisher_node, broadcaster, 1.0)
    first_value = _spin_until(
        executor,
        lambda: _lookup_translation_x(buffer),
        timeout_sec=2.0,
    )
    assert first_value == pytest.approx(1.0)

    _publish_static_transform(publisher_node, broadcaster, 2.0)
    updated_value = _spin_until(
        executor,
        lambda: (
            value
            if (value := _lookup_translation_x(buffer)) == pytest.approx(2.0)
            else None
        ),
        timeout_sec=3.0,
    )
    assert updated_value == pytest.approx(2.0)


def test_static_tf_republish_is_visible_to_late_listener(ros_graph):
    executor, create_node = ros_graph
    publisher_node = create_node("tf_pub")
    first_listener_node = create_node("tf_listener")

    first_buffer = Buffer()
    TransformListener(first_buffer, first_listener_node, spin_thread=False)
    broadcaster = StaticTransformBroadcaster(publisher_node)

    _publish_static_transform(publisher_node, broadcaster, 1.0)
    assert _spin_until(
        executor,
        lambda: _lookup_translation_x(first_buffer),
        timeout_sec=2.0,
    ) == pytest.approx(1.0)

    _publish_static_transform(publisher_node, broadcaster, 3.0)
    assert _spin_until(
        executor,
        lambda: (
            value
            if (value := _lookup_translation_x(first_buffer))
            == pytest.approx(3.0)
            else None
        ),
        timeout_sec=3.0,
    ) == pytest.approx(3.0)

    late_listener_node = create_node("tf_listener_late")
    late_buffer = Buffer()
    TransformListener(late_buffer, late_listener_node, spin_thread=False)

    late_value = _spin_until(
        executor,
        lambda: _lookup_translation_x(late_buffer),
        timeout_sec=2.0,
    )
    assert late_value == pytest.approx(3.0)
