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

import importlib
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robo_orchard_data_ros2.tf.config import TFConfig


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeClock:
    class _Now:
        @staticmethod
        def to_msg():
            return "stamp"

    @staticmethod
    def now():
        return FakeClock._Now()


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.logger = FakeLogger()
        self.services = []
        self.timers = []
        self.parameters = {}

    def create_service(self, service_type, name, callback):
        self.services.append(
            types.SimpleNamespace(
                service_type=service_type,
                name=name,
                callback=callback,
            )
        )
        return self.services[-1]

    def create_timer(self, interval, callback):
        self.timers.append((interval, callback))
        return self.timers[-1]

    def declare_parameter(self, name, value):
        self.parameters[name] = value
        return types.SimpleNamespace(name=name, value=value)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return FakeClock()


class FakeTransformStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id=None)
        self.child_frame_id = None
        self.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=None, y=None, z=None),
            rotation=types.SimpleNamespace(x=None, y=None, z=None, w=None),
        )


class FakeStaticTransformBroadcaster:
    instances = []

    def __init__(self, node):
        self.node = node
        self.transforms = []
        self.__class__.instances.append(self)

    def sendTransform(self, transforms):  # noqa: N802
        self.transforms.append(list(transforms))


class FakeDynamicTransformBroadcaster(FakeStaticTransformBroadcaster):
    instances = []


def _install_node_stubs(monkeypatch):
    fake_rclpy = types.SimpleNamespace(
        init=lambda args=None: None,
        shutdown=lambda: None,
        spin=lambda node: None,
    )
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(
        sys.modules,
        "rclpy.node",
        types.SimpleNamespace(
            Node=FakeNode,
            ParameterDescriptor=lambda description=None: types.SimpleNamespace(
                description=description
            ),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        types.SimpleNamespace(TransformStamped=FakeTransformStamped),
    )
    monkeypatch.setitem(
        sys.modules,
        "tf2_ros.static_transform_broadcaster",
        types.SimpleNamespace(
            StaticTransformBroadcaster=FakeStaticTransformBroadcaster
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "tf2_ros.transform_broadcaster",
        types.SimpleNamespace(
            TransformBroadcaster=FakeDynamicTransformBroadcaster
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "robo_orchard_data_msg_ros2.srv",
        types.SimpleNamespace(SetStaticTransforms=object),
    )


def _reload_tf_node_module():
    sys.modules.pop("robo_orchard_data_ros2.tf.node", None)
    return importlib.import_module("robo_orchard_data_ros2.tf.node")


def _make_request(directory):
    return types.SimpleNamespace(directory=directory)


def test_tf_publisher_starts_empty(monkeypatch):
    _install_node_stubs(monkeypatch)
    FakeStaticTransformBroadcaster.instances.clear()
    FakeDynamicTransformBroadcaster.instances.clear()

    tf_node = _reload_tf_node_module()
    node = tf_node.TFPublisherNode()

    assert len(FakeStaticTransformBroadcaster.instances) == 1
    assert len(FakeDynamicTransformBroadcaster.instances) == 0
    assert node.timers == []
    assert node.services[0].name == "set_static_transforms"
    assert len(FakeStaticTransformBroadcaster.instances[0].transforms) == 0
    assert node.logger.warnings == [
        "TF publisher starting empty: no tf is published"
    ]


def test_tf_publisher_declares_unique_startup_id_per_instance(monkeypatch):
    _install_node_stubs(monkeypatch)
    FakeStaticTransformBroadcaster.instances.clear()

    tf_node = _reload_tf_node_module()
    first = tf_node.TFPublisherNode()
    second = tf_node.TFPublisherNode()

    assert "startup_id" in first.parameters
    assert "startup_id" in second.parameters
    assert isinstance(first.parameters["startup_id"], str)
    assert first.parameters["startup_id"] != second.parameters["startup_id"]


def test_set_static_transforms_loads_all_json_from_directory(
    monkeypatch, tmp_path
):
    _install_node_stubs(monkeypatch)
    FakeStaticTransformBroadcaster.instances.clear()

    tf_node = _reload_tf_node_module()

    tf_by_file = {
        str(tmp_path / "left.json"): TFConfig(
            parent_frame_id="tool0",
            child_frame_id="camera0",
            xyz=(0.1, 0.2, 0.3),
            quat=(0.4, 0.5, 0.6, 0.7),
            scalar_first=False,
        ),
        str(tmp_path / "right.json"): TFConfig(
            parent_frame_id="base",
            child_frame_id="camera1",
            xyz=(1.0, 2.0, 3.0),
            quat=(0.0, 0.0, 0.0, 1.0),
            scalar_first=False,
        ),
    }
    (tmp_path / "left.json").write_text("{}")
    (tmp_path / "right.json").write_text("{}")

    load_calls = []

    def fake_load(result_file):
        load_calls.append(result_file)
        return tf_by_file[result_file]

    monkeypatch.setattr(tf_node, "load_tf_config_from_result_file", fake_load)

    node = tf_node.TFPublisherNode()
    response = types.SimpleNamespace(success=None, message=None)

    returned = node._set_static_transforms_service_callback(
        _make_request(str(tmp_path)), response
    )

    assert returned.success is True
    assert returned.message == ""
    assert set(load_calls) == set(tf_by_file.keys())
    assert len(FakeStaticTransformBroadcaster.instances[0].transforms) == 1
    published_children = {
        t.child_frame_id
        for t in FakeStaticTransformBroadcaster.instances[0].transforms[-1]
    }
    assert published_children == {"camera0", "camera1"}


def test_set_static_transforms_returns_failure_on_invalid_directory(
    monkeypatch, tmp_path
):
    _install_node_stubs(monkeypatch)
    FakeStaticTransformBroadcaster.instances.clear()

    tf_node = _reload_tf_node_module()
    node = tf_node.TFPublisherNode()
    response = types.SimpleNamespace(success=None, message=None)

    returned = node._set_static_transforms_service_callback(
        _make_request(str(tmp_path / "nonexistent")), response
    )

    assert returned.success is False
    assert "not a directory" in returned.message.lower()
    assert len(FakeStaticTransformBroadcaster.instances[0].transforms) == 0


def test_set_static_transforms_returns_failure_on_oserror(
    monkeypatch, tmp_path
):
    _install_node_stubs(monkeypatch)
    FakeStaticTransformBroadcaster.instances.clear()

    tf_node = _reload_tf_node_module()

    (tmp_path / "left.json").write_text("{}")

    def fake_load(result_file):
        raise OSError("permission denied")

    monkeypatch.setattr(tf_node, "load_tf_config_from_result_file", fake_load)

    node = tf_node.TFPublisherNode()
    response = types.SimpleNamespace(success=None, message=None)

    returned = node._set_static_transforms_service_callback(
        _make_request(str(tmp_path)), response
    )

    assert returned.success is False
    assert "permission denied" in returned.message
    assert len(FakeStaticTransformBroadcaster.instances[0].transforms) == 0
