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

import time
from types import SimpleNamespace as Namespace

import numpy as np
import pytest

from robo_orchard_wuji_glove_ros2.sdk_client import (
    SDK_STREAM_SPECS,
    SdkClientConfig,
    WujiSdkClient,
    WujiSdkRetargeter,
)


class _Subscription:
    def __init__(self, callback, on_error):
        self.callback = callback
        self.on_error = on_error
        self.closed = False

    def close(self):
        self.closed = True


class _Resource:
    def __init__(self, value=None):
        self.value = value
        self.subscriptions = []
        self.set_values = []
        self.subscribe_error = None

    def get(self):
        return self.value

    def set(self, value):
        self.set_values.append(value)
        self.value = value

    def subscribe_with_callback(self, callback, on_error=None):
        subscription = _Subscription(callback, on_error)
        self.subscriptions.append(subscription)
        if self.subscribe_error is not None:
            on_error(self.subscribe_error)
        return subscription


class _Glove:
    def __init__(self):
        self.is_connected = True
        self.sync_calls = 0
        self.sync_error = None
        self.sync_result = Namespace(
            offset_us=1_700_000_000_000_000,
            round_trip_us=1_000,
            synced_at_us=time.time_ns() // 1_000,
        )
        self.resources = {
            name: _Resource()
            for name in (
                "tactile",
                "tactile_zones",
                "tactile_binary",
                "tactile_residual",
                "emf_poses",
                "tip_poses",
                "hand_joint_angles",
                "hand_skeleton",
                "tactile_point_cloud",
                "imu_palm",
            )
        }
        self.resources.update(
            {
                "sn": _Resource("WG1KA001"),
                "version": _Resource("0.11.0"),
                "ip": _Resource("192.168.1.120"),
                "port": _Resource(50000),
                "hand_side": _Resource("right"),
                "emf_poses_rate_divider": _Resource(1),
                "hand_model_path": _Resource("/stale/model.urdf"),
            }
        )

    def __getattr__(self, name):
        if name in self.resources:
            return lambda: self.resources[name]
        raise AttributeError(name)

    def sync_time(self):
        self.sync_calls += 1
        if self.sync_error is not None:
            raise self.sync_error
        return self.sync_result


class _Manager:
    def __init__(self, glove):
        self.glove = glove
        self.user = {
            "user_id": "prior-user",
            "display_name": "Prior",
        }
        self.connect_args = None
        self.connect_error = None
        self.disconnected = []
        self.scan_calls = 0
        self.scan_devices = [
            Namespace(sn="WG1KA001", device_type="wuji-glove")
        ]
        self.tf_resource = _Resource()
        self.tf_static_resource = _Resource()

    def current_user(self):
        return self.user

    def switch_to_default_user(self):
        self.user = {"user_id": "", "display_name": "Default"}
        return self.user

    def switch_user(self, user_id):
        self.user = {"user_id": user_id, "display_name": user_id}
        return self.user

    def list_users(self):
        return [
            {"user_id": "alice-id", "display_name": "Alice"},
            {"user_id": "bob-id", "display_name": "Bob"},
        ]

    def scan(self):
        self.scan_calls += 1
        return self.scan_devices

    def connect(self, **kwargs):
        self.connect_args = kwargs
        if self.connect_error is not None:
            raise self.connect_error
        self.glove.is_connected = True
        return self.glove

    def disconnect(self, device_name):
        self.disconnected.append(device_name)
        self.glove.is_connected = False

    def tf(self):
        return self.tf_resource

    def tf_static(self):
        return self.tf_static_resource


class _ConnectOptions:
    def __init__(
        self,
        timeout_ms=1000,
        retry_count=3,
        enable_bridge=True,
        auto_time_sync_interval_ms=30000,
    ):
        self.timeout_ms = timeout_ms
        self.retry_count = retry_count
        self.enable_bridge = enable_bridge
        self.auto_time_sync_interval_ms = auto_time_sync_interval_ms


def _sdk_and_manager():
    glove = _Glove()
    manager = _Manager(glove)

    class _SdkManager:
        @staticmethod
        def instance():
            return manager

    sdk = Namespace(
        SdkManager=_SdkManager,
        DeviceType=Namespace(
            Unknown="unknown",
            WujiGlove="wuji-glove",
            WujiHand2="wuji-hand-2",
        ),
        ConnectOptions=_ConnectOptions,
        WujiGlove=_Glove,
        Handedness=Namespace(Left="left-enum", Right="right-enum"),
        HandModel=Namespace(WujiHand="hand-v1", WujiHand2="hand-v2"),
        set_log_level=lambda level: None,
    )
    return sdk, manager, glove


def _assert_connect_args(manager, serial_number):
    assert manager.connect_args["device_name"] == "glove"
    assert manager.connect_args["sn"] == serial_number
    assert set(manager.connect_args) == {"device_name", "sn", "options"}
    options = manager.connect_args["options"]
    assert options.timeout_ms == 1000
    assert options.retry_count == 3


def _config(**overrides):
    values = {
        "serial_number": "",
        "hand_side": "right",
        "device_name": "glove",
        "frame_prefix": "glove",
        "sdk_user": "default",
        "hand_model_path": "",
        "sdk_log_level": "warn",
        "emf_poses_rate_divider": 4,
        "time_sync_max_age_s": 5.0,
        "time_sync_max_rtt_ms": 100.0,
        "enabled_streams": frozenset(
            spec.name for spec in SDK_STREAM_SPECS if spec.enabled_by_default
        ),
        "publish_transforms": True,
    }
    values.update(overrides)
    return SdkClientConfig(**values)


def _config_without_streams(**overrides):
    values = {
        "enabled_streams": frozenset(),
        "publish_transforms": False,
    }
    values.update(overrides)
    return _config(**values)


def _write_hand_model_root(root, hand_side="right"):
    root.mkdir(parents=True)
    model = root / "alice-id" / "models" / f"{hand_side}_hand.urdf"
    model.parent.mkdir(parents=True)
    model.write_text(
        f"<robot name='operator_{hand_side}'/>",
        encoding="utf-8",
    )
    return model.resolve()


def test_client_connects_by_side_and_opens_requested_streams():
    sdk, manager, glove = _sdk_and_manager()
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")
    callbacks = {
        name: lambda frame: None
        for name in (
            "tactile",
            "tactile_zones",
            "emf_poses",
            "tip_poses",
            "hand_joint_angles",
            "hand_skeleton",
            "tactile_point_cloud",
            "imu_palm",
            "transforms",
            "static_transforms",
        )
    }
    errors = []

    info = client.connect(_config(), callbacks, errors.append)

    _assert_connect_args(manager, "WG1KA001")
    assert manager.scan_calls == 1
    assert glove.resources["emf_poses_rate_divider"].value == 4
    assert client.connected
    assert info.serial_number == "WG1KA001"
    assert info.sdk_version == "2026.7.21"
    assert glove.sync_calls == 1
    assert len(client._subscriptions) == 10

    glove.resources["tactile"].subscriptions[0].on_error("closed")
    assert not client.connected
    assert errors == ["tactile: closed"]

    subscriptions = list(client._subscriptions)
    client.disconnect()
    assert all(subscription.closed for subscription in subscriptions)
    assert manager.disconnected == ["glove"]
    assert manager.current_user()["user_id"] == "prior-user"

    subscriptions[0].on_error("late close callback")
    assert errors == ["tactile: closed"]


def test_client_selects_serial_number_and_named_sdk_user():
    sdk, manager, glove = _sdk_and_manager()
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    client.connect(
        _config(
            serial_number="WG1KA001",
            sdk_user="Alice",
            enabled_streams=frozenset(),
            publish_transforms=False,
        ),
        {},
        lambda message: None,
    )

    _assert_connect_args(manager, "WG1KA001")
    assert manager.scan_calls == 0
    assert manager.current_user()["user_id"] == "alice-id"
    assert glove.resources["hand_model_path"].set_values == []
    client.disconnect()


@pytest.mark.parametrize("serials", [[], ["WG1KA001", "WG1KA002"]])
def test_client_requires_exactly_one_matching_glove(serials):
    sdk, manager, _ = _sdk_and_manager()
    manager.scan_devices = [
        Namespace(sn=serial, device_type=sdk.DeviceType.WujiGlove)
        for serial in serials
    ]
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    with pytest.raises(
        RuntimeError,
        match=(
            "exactly one matching glove must be discoverable.*"
            "Set serial_number explicitly"
        ),
    ):
        client.connect(_config_without_streams(), {}, lambda message: None)

    assert manager.connect_args is None
    assert not client.connected


def test_client_filters_other_device_types_before_selecting_by_side():
    sdk, manager, _ = _sdk_and_manager()
    manager.scan_devices.append(
        Namespace(sn="WH2KA001", device_type=sdk.DeviceType.WujiHand2)
    )
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    client.connect(_config_without_streams(), {}, lambda message: None)

    _assert_connect_args(manager, "WG1KA001")
    client.disconnect()


def test_client_accepts_unique_untyped_glove_candidate():
    sdk, manager, _ = _sdk_and_manager()
    manager.scan_devices = [
        Namespace(sn="WG1KA001", device_type=sdk.DeviceType.Unknown)
    ]
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    client.connect(_config_without_streams(), {}, lambda message: None)

    _assert_connect_args(manager, "WG1KA001")
    client.disconnect()


def test_client_requires_serial_for_known_and_untyped_candidates():
    sdk, manager, _ = _sdk_and_manager()
    manager.scan_devices.append(
        Namespace(sn="WG1KA002", device_type=sdk.DeviceType.Unknown)
    )
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    with pytest.raises(
        RuntimeError,
        match="exactly one matching glove must be discoverable.*found 2",
    ):
        client.connect(_config_without_streams(), {}, lambda message: None)

    assert manager.connect_args is None
    assert not client.connected


def test_client_reports_failure_for_selected_glove_serial():
    sdk, manager, _ = _sdk_and_manager()
    manager.connect_error = RuntimeError("selected device disappeared")
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    with pytest.raises(
        RuntimeError, match="selected right Wuji Glove"
    ) as raised:
        client.connect(_config_without_streams(), {}, lambda message: None)

    assert "selected device disappeared" in str(raised.value)
    assert not client.connected


def test_client_rejects_invalid_time_sync_before_subscribing():
    sdk, manager, glove = _sdk_and_manager()
    glove.sync_result = Namespace(
        offset_us=0,
        round_trip_us=1_000,
        synced_at_us=0,
    )
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    with pytest.raises(RuntimeError, match="invalid time synchronization"):
        client.connect(_config(), {}, lambda message: None)

    assert glove.sync_calls == 1
    assert not client._subscriptions
    assert manager.disconnected == ["glove"]
    assert not client.connected


@pytest.mark.parametrize(
    "sync_result",
    [
        Namespace(
            offset_us=0,
            round_trip_us=1_000,
            synced_at_us=(time.time_ns() // 1_000) - 10_000_000,
        ),
        Namespace(
            offset_us=0,
            round_trip_us=200_000,
            synced_at_us=time.time_ns() // 1_000,
        ),
    ],
)
def test_client_rejects_stale_or_high_rtt_time_sync(sync_result):
    sdk, manager, glove = _sdk_and_manager()
    glove.sync_result = sync_result
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    with pytest.raises(RuntimeError, match="invalid time synchronization"):
        client.connect(_config(), {}, lambda message: None)

    assert not client._subscriptions
    assert manager.disconnected == ["glove"]


@pytest.mark.parametrize("sdk_user", ["alice-id", "Alice"])
def test_client_resolves_registry_user_and_external_model(tmp_path, sdk_user):
    sdk, manager, glove = _sdk_and_manager()
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")
    model_root = tmp_path / "users"
    hand_model = _write_hand_model_root(model_root)

    info = client.connect(
        _config_without_streams(
            sdk_user=sdk_user,
            hand_model_path=str(model_root),
        ),
        {},
        lambda message: None,
    )

    expected = str(hand_model)
    assert manager.current_user()["user_id"] == "alice-id"
    assert glove.resources["hand_model_path"].value == expected
    assert glove.resources["hand_model_path"].set_values == [expected]
    assert info.hand_model_path == expected
    assert info.sdk_user == "alice-id"
    client.disconnect()


def test_client_uses_sdk_managed_model_without_external_override(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("HOME", str(tmp_path))
    sdk, manager, glove = _sdk_and_manager()
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")
    model_root = tmp_path / ".wuji" / "sdk" / "users"
    hand_model = _write_hand_model_root(model_root)

    info = client.connect(
        _config_without_streams(
            sdk_user="Alice",
            hand_model_path=str(model_root),
        ),
        {},
        lambda message: None,
    )

    assert manager.current_user()["user_id"] == "alice-id"
    assert glove.resources["hand_model_path"].set_values == []
    assert info.hand_model_path == str(hand_model)
    client.disconnect()


def test_client_rejects_default_user_for_external_model(tmp_path):
    sdk, manager, _ = _sdk_and_manager()
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")
    model_root = tmp_path / "users"
    _write_hand_model_root(model_root)

    with pytest.raises(RuntimeError, match="requires a named Wuji SDK user"):
        client.connect(
            _config_without_streams(
                sdk_user="default",
                hand_model_path=str(model_root),
            ),
            {},
            lambda message: None,
        )

    assert manager.connect_args is None


def test_client_resolves_left_hand_model(tmp_path):
    sdk, manager, glove = _sdk_and_manager()
    glove.resources["hand_side"].value = "left"
    manager.scan_devices = [
        Namespace(sn="WG1JA001", device_type=sdk.DeviceType.WujiGlove)
    ]
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")
    model_root = tmp_path / "users"
    hand_model = _write_hand_model_root(model_root, hand_side="left")

    info = client.connect(
        _config_without_streams(
            hand_side="left",
            sdk_user="Alice",
            hand_model_path=str(model_root),
        ),
        {},
        lambda message: None,
    )

    assert manager.current_user()["user_id"] == "alice-id"
    assert info.hand_model_path == str(hand_model)
    client.disconnect()


def test_client_fails_when_stream_ends_during_subscription():
    sdk, manager, glove = _sdk_and_manager()
    glove.resources["tactile"].subscribe_error = "closed"
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")
    errors = []

    with pytest.raises(RuntimeError, match="tactile: closed"):
        client.connect(
            _config(),
            {
                spec.name: lambda frame: None
                for spec in SDK_STREAM_SPECS
                if spec.enabled_by_default
            }
            | {
                "transforms": lambda frame: None,
                "static_transforms": lambda frame: None,
            },
            errors.append,
        )

    assert not client.connected
    assert errors == ["tactile: closed"]
    assert manager.disconnected == ["glove"]
    assert all(
        subscription.closed
        for subscription in glove.resources["tactile"].subscriptions
    )


def test_client_disconnects_device_with_unexpected_type():
    sdk, manager, _ = _sdk_and_manager()
    manager.glove = Namespace(is_connected=True)
    client = WujiSdkClient(sdk, sdk_version="2026.7.21")

    with pytest.raises(RuntimeError, match="not a Wuji Glove"):
        client.connect(_config_without_streams(), {}, lambda message: None)

    assert manager.disconnected == ["glove"]
    assert manager.current_user()["user_id"] == "prior-user"


def test_retargeter_uses_sdk_model_and_validates_output():
    calls = []

    class _Session:
        def step(self, keypoints):
            calls.append(keypoints.copy())
            return np.arange(20, dtype=np.float32)

        def reset(self):
            calls.append("reset")

    class _RetargetSession:
        @staticmethod
        def for_hand(model, side):
            assert model == "hand-v1"
            assert side == "right-enum"
            return _Session()

    sdk, _, _ = _sdk_and_manager()
    sdk.RetargetSession = _RetargetSession
    retargeter = WujiSdkRetargeter("wuji_hand", "right", sdk)

    result = retargeter.step(np.ones((21, 3), dtype=np.float64))
    retargeter.reset()

    assert result.shape == (20,)
    assert calls[0].dtype == np.float32
    assert calls[1] == "reset"
