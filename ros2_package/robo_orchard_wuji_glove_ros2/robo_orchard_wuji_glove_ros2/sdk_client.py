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

from __future__ import annotations
import contextlib
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

SUPPORTED_WUJI_SDK_VERSION = "2026.7.21"
SDK_CONNECT_TIMEOUT_MS = 1_000
SDK_CONNECT_RETRY_COUNT = 3
EARLIEST_UTC_TIMESTAMP_US = 1_577_836_800_000_000
LATEST_ROS_TIMESTAMP_US = 2_147_483_647_999_999


def load_wuji_sdk() -> tuple[ModuleType, str]:
    """Load the tested Wuji Python SDK and report actionable failures."""
    try:
        import wuji_sdk
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "wuji-sdk is not installed in the ROS Python environment; run "
            f"'python3 -m pip install wuji-sdk=={SUPPORTED_WUJI_SDK_VERSION}'"
        ) from error

    try:
        version = metadata.version("wuji-sdk")
    except metadata.PackageNotFoundError as error:
        raise RuntimeError(
            "wuji_sdk can be imported, but its distribution metadata is "
            "missing; reinstall the wuji-sdk wheel"
        ) from error

    if version != SUPPORTED_WUJI_SDK_VERSION:
        raise RuntimeError(
            f"unsupported wuji-sdk version {version}; this package is tested "
            f"with {SUPPORTED_WUJI_SDK_VERSION}"
        )
    return wuji_sdk, version


@dataclass(frozen=True)
class SdkClientConfig:
    """Wuji SDK connection and stream configuration."""

    serial_number: str
    hand_side: str
    device_name: str
    frame_prefix: str
    sdk_user: str
    hand_model_path: str
    sdk_log_level: str
    emf_poses_rate_divider: int
    time_sync_max_age_s: float
    time_sync_max_rtt_ms: float
    enabled_streams: frozenset[str]
    publish_transforms: bool


@dataclass(frozen=True)
class GloveDeviceInfo:
    """Connection metadata published independently of vendor SDK objects."""

    serial_number: str = ""
    device_name: str = ""
    source_namespace: str = ""
    firmware_version: str = ""
    sdk_version: str = ""
    handedness: str = ""
    ip_address: str = ""
    data_port: int = 0
    connected: bool = False
    sdk_user: str = "default"
    hand_model_path: str = ""


@dataclass(frozen=True)
class SdkStreamSpec:
    """One configurable Wuji SDK stream and its ROS topic."""

    name: str
    topic: str
    enabled_by_default: bool

    @property
    def parameter_name(self) -> str:
        """Return the ROS parameter controlling this stream."""
        return f"publish_{self.name}"


SDK_STREAM_SPECS = (
    SdkStreamSpec("tactile", "tactile", True),
    SdkStreamSpec("tactile_zones", "tactile_zones", True),
    SdkStreamSpec("tactile_binary", "tactile_binary", False),
    SdkStreamSpec("tactile_residual", "tactile_residual", False),
    SdkStreamSpec("emf_poses", "emf_poses", True),
    SdkStreamSpec("tip_poses", "tip_poses", True),
    SdkStreamSpec("hand_joint_angles", "hand_joint_angles", True),
    SdkStreamSpec("hand_skeleton", "hand_skeleton", True),
    SdkStreamSpec("tactile_point_cloud", "tactile_point_cloud", True),
    SdkStreamSpec("imu_palm", "imu/palm", True),
)


class WujiSdkClient:
    """Own one Wuji Glove connection and all of its callback streams."""

    def __init__(
        self,
        sdk_module: ModuleType | Any | None = None,
        sdk_version: str | None = None,
    ) -> None:
        if sdk_module is None:
            sdk_module, sdk_version = load_wuji_sdk()
        self._sdk = sdk_module
        self._sdk_version = sdk_version or SUPPORTED_WUJI_SDK_VERSION
        self._manager = self._sdk.SdkManager.instance()
        self._operation_lock = threading.RLock()
        self._state_lock = threading.Lock()
        self._subscriptions: list[Any] = []
        self._glove: Any | None = None
        self._device_name = ""
        self._connection_token: object | None = None
        self._stream_error: str | None = None
        self._previous_user_id: str | None = None
        self._healthy = False

    @property
    def connected(self) -> bool:
        """Return whether the device and all requested streams are healthy."""
        with self._state_lock:
            glove = self._glove
            healthy = self._healthy
        if glove is None or not healthy:
            return False
        try:
            return bool(glove.is_connected)
        except Exception:
            return False

    def connect(
        self,
        config: SdkClientConfig,
        callbacks: Mapping[str, Callable[[Any], None]],
        on_error: Callable[[str], None],
    ) -> GloveDeviceInfo:
        """Connect and return only after every requested stream is active."""
        with self._operation_lock:
            self.disconnect()
            self._sdk.set_log_level(config.sdk_log_level)
            previous_user = self._manager.current_user()
            self._previous_user_id = previous_user["user_id"]
            connection_token = object()
            with self._state_lock:
                self._connection_token = connection_token
                self._stream_error = None

            try:
                selected_user = self._select_sdk_user(config.sdk_user)
                hand_model_path = self._resolve_hand_model_path(
                    config.hand_model_path,
                    selected_user,
                    config.hand_side,
                )
                connect_args: dict[str, Any] = {
                    "device_name": config.device_name,
                    "sn": self._select_glove_serial(config),
                    "options": self._sdk.ConnectOptions(
                        timeout_ms=SDK_CONNECT_TIMEOUT_MS,
                        retry_count=SDK_CONNECT_RETRY_COUNT,
                    ),
                }

                try:
                    glove = self._manager.connect(**connect_args)
                except Exception as error:
                    if not config.serial_number:
                        raise RuntimeError(
                            "failed to connect the selected "
                            f"{config.hand_side} "
                            f"Wuji Glove {connect_args['sn']!r}; set "
                            "serial_number explicitly if the device identity "
                            f"has changed. SDK error: {error}"
                        ) from error
                    raise
                with self._state_lock:
                    self._device_name = config.device_name
                if not isinstance(glove, self._sdk.WujiGlove):
                    actual_type = type(glove).__name__
                    raise RuntimeError(
                        f"selected device is not a Wuji Glove: {actual_type}"
                    )
                with self._state_lock:
                    self._glove = glove

                actual_side = str(glove.hand_side().get()).lower()
                if actual_side != config.hand_side:
                    raise RuntimeError(
                        f"connected {actual_side} glove while "
                        f"{config.hand_side} "
                        "was requested"
                    )
                active_hand_model_path = self._configure_hand_model(
                    glove,
                    selected_user,
                    hand_model_path,
                    config.hand_side,
                )
                glove.emf_poses_rate_divider().set(
                    config.emf_poses_rate_divider
                )
                self._verify_time_sync(glove, config)

                self._open_subscriptions(
                    glove,
                    config,
                    callbacks,
                    on_error,
                    connection_token,
                )
                info = GloveDeviceInfo(
                    serial_number=str(glove.sn().get()),
                    device_name=config.device_name,
                    firmware_version=str(glove.version().get()),
                    sdk_version=self._sdk_version,
                    handedness=actual_side,
                    ip_address=self._optional_get(glove.ip),
                    data_port=self._optional_int_get(glove.port),
                    connected=True,
                    sdk_user=(
                        str(selected_user["user_id"])
                        if selected_user["user_id"]
                        else "default"
                    ),
                    hand_model_path=active_hand_model_path,
                )
                with self._state_lock:
                    stream_error = self._stream_error
                    if stream_error is None:
                        self._healthy = True
                if stream_error is not None:
                    raise RuntimeError(
                        "Wuji Glove stream failed while connecting: "
                        f"{stream_error}"
                    )
                return info
            except Exception:
                self.disconnect()
                raise

    def _select_glove_serial(self, config: SdkClientConfig) -> str:
        if config.serial_number:
            return config.serial_number

        side_marker = "J" if config.hand_side == "left" else "K"
        try:
            devices = self._manager.scan()
        except Exception as error:
            raise RuntimeError(
                f"failed to discover {config.hand_side} Wuji Gloves: {error}"
            ) from error
        candidate_types = (
            self._sdk.DeviceType.WujiGlove,
            self._sdk.DeviceType.Unknown,
        )
        candidates = sorted(
            {
                str(device.sn)
                for device in devices
                if device.device_type in candidate_types
                and len(str(device.sn)) > 3
                and str(device.sn)[3].upper() == side_marker
            }
        )
        if len(candidates) != 1:
            discovered = ", ".join(candidates) or "none"
            raise RuntimeError(
                f"failed to select a {config.hand_side} Wuji Glove; exactly "
                "one matching glove must be discoverable, but found "
                f"{len(candidates)} ({discovered}). Set serial_number "
                "explicitly when multiple matching gloves are present"
            )
        return candidates[0]

    @staticmethod
    def _verify_time_sync(glove: Any, config: SdkClientConfig) -> None:
        try:
            result = glove.sync_time()
            synced_at_us = int(result.synced_at_us)
            round_trip_us = int(result.round_trip_us)
        except Exception as error:
            raise RuntimeError(
                f"Wuji Glove time synchronization failed: {error}"
            ) from error
        host_time_us = time.time_ns() // 1_000
        sync_age_us = abs(host_time_us - synced_at_us)
        if (
            not (
                EARLIEST_UTC_TIMESTAMP_US
                <= synced_at_us
                <= LATEST_ROS_TIMESTAMP_US
            )
            or round_trip_us < 0
            or sync_age_us > config.time_sync_max_age_s * 1_000_000
            or round_trip_us > config.time_sync_max_rtt_ms * 1_000
        ):
            raise RuntimeError(
                "Wuji Glove returned an invalid time synchronization result: "
                f"synced_at_us={synced_at_us}, sync_age_us={sync_age_us}, "
                f"round_trip_us={round_trip_us}"
            )

    def disconnect(self) -> None:
        """Close callbacks, disconnect the glove, and restore SDK user."""
        with self._operation_lock:
            with self._state_lock:
                subscriptions = self._subscriptions
                self._subscriptions = []
                device_name = self._device_name
                self._device_name = ""
                self._glove = None
                self._connection_token = None
                self._stream_error = None
                self._healthy = False
                previous_user_id = self._previous_user_id
                self._previous_user_id = None

            for subscription in reversed(subscriptions):
                with contextlib.suppress(Exception):
                    subscription.close()
            if device_name:
                with contextlib.suppress(Exception):
                    self._manager.disconnect(device_name)
            if previous_user_id is not None:
                with contextlib.suppress(Exception):
                    self._manager.switch_user(previous_user_id)

    def _select_sdk_user(self, requested: str) -> Mapping[str, Any]:
        if not requested or requested.lower() == "default":
            return self._manager.switch_to_default_user()

        users = self._manager.list_users()
        id_matches = [user for user in users if user["user_id"] == requested]
        if id_matches:
            return self._manager.switch_user(id_matches[0]["user_id"])
        name_matches = [
            user for user in users if user["display_name"] == requested
        ]
        if len(name_matches) != 1:
            raise RuntimeError(
                f"sdk_user {requested!r} did not resolve to exactly one SDK "
                "user ID or display name"
            )
        return self._manager.switch_user(name_matches[0]["user_id"])

    @staticmethod
    def _resolve_hand_model_path(
        requested_root: str,
        selected_user: Mapping[str, Any],
        hand_side: str,
    ) -> str:
        value = requested_root.strip()
        if not value:
            return ""
        try:
            root = Path(value).expanduser().resolve(strict=True)
        except (OSError, RuntimeError) as error:
            raise RuntimeError(
                f"hand_model_path does not resolve to a directory: {value}"
            ) from error
        if not root.is_dir():
            raise RuntimeError(f"hand_model_path is not a directory: {root}")

        user_id = selected_user.get("user_id")
        if not isinstance(user_id, str) or not user_id:
            raise RuntimeError(
                "hand_model_path requires a named Wuji SDK user; the default "
                "user always uses the SDK built-in hand model"
            )

        model_path = (
            root / user_id / "models" / f"{hand_side}_hand.urdf"
        ).resolve(strict=False)
        try:
            model_path.relative_to(root)
        except ValueError as error:
            raise RuntimeError(
                f"unsafe Wuji SDK user_id: {user_id!r}"
            ) from error
        if not model_path.is_file():
            raise RuntimeError(
                f"calibrated {hand_side} hand model does not exist: "
                f"{model_path}"
            )
        return str(model_path)

    @staticmethod
    def _configure_hand_model(
        glove: Any,
        selected_user: Mapping[str, Any],
        hand_model_path: str,
        hand_side: str,
    ) -> str:
        if not selected_user["user_id"]:
            return ""

        if not hand_model_path:
            return ""

        managed_model_path = (
            Path.home()
            / ".wuji"
            / "sdk"
            / "users"
            / str(selected_user["user_id"])
            / "models"
            / f"{hand_side}_hand.urdf"
        ).resolve(strict=False)
        if Path(hand_model_path) == managed_model_path:
            # Selecting the named SDK user before connecting makes the SDK
            # load this managed model. The setter is only for external URDFs.
            return hand_model_path

        resource = glove.hand_model_path()
        resource.set(hand_model_path)
        active = str(resource.get())
        try:
            active_path = Path(active).expanduser().resolve(strict=False)
        except (OSError, RuntimeError) as error:
            raise RuntimeError(
                f"Wuji SDK returned an invalid hand model path: {active}"
            ) from error
        if active_path != Path(hand_model_path):
            raise RuntimeError(
                "Wuji SDK did not activate the requested hand model: "
                f"requested={hand_model_path}, active={active}"
            )
        return str(active_path)

    def _open_subscriptions(
        self,
        glove: Any,
        config: SdkClientConfig,
        callbacks: Mapping[str, Callable[[Any], None]],
        on_error: Callable[[str], None],
        connection_token: object,
    ) -> None:
        for spec in SDK_STREAM_SPECS:
            if spec.name in config.enabled_streams:
                self._subscribe(
                    spec.name,
                    getattr(glove, spec.name)(),
                    callbacks[spec.name],
                    on_error,
                    connection_token,
                )

        if config.publish_transforms:
            self._subscribe(
                "transforms",
                self._manager.tf(),
                callbacks["transforms"],
                on_error,
                connection_token,
            )
            self._subscribe(
                "static_transforms",
                self._manager.tf_static(),
                callbacks["static_transforms"],
                on_error,
                connection_token,
            )

    def _subscribe(
        self,
        stream_name: str,
        resource: Any,
        callback: Callable[[Any], None],
        on_error: Callable[[str], None],
        connection_token: object,
    ) -> None:
        def stream_error(message: str) -> None:
            error = f"{stream_name}: {message}"
            with self._state_lock:
                if self._connection_token is not connection_token:
                    return
                self._healthy = False
                if self._stream_error is None:
                    self._stream_error = error
            on_error(error)

        subscription = resource.subscribe_with_callback(
            callback=callback, on_error=stream_error
        )
        with self._state_lock:
            self._subscriptions.append(subscription)

    @staticmethod
    def _optional_get(resource_factory: Callable[[], Any]) -> str:
        try:
            return str(resource_factory().get())
        except Exception:
            return ""

    @staticmethod
    def _optional_int_get(resource_factory: Callable[[], Any]) -> int:
        try:
            return int(resource_factory().get())
        except Exception:
            return 0


class WujiSdkRetargeter:
    """Thin validated wrapper around ``wuji_sdk.RetargetSession``."""

    def __init__(
        self,
        hand_model: str,
        hand_side: str,
        sdk_module: ModuleType | Any | None = None,
    ) -> None:
        if sdk_module is None:
            sdk_module, _ = load_wuji_sdk()
        model = {
            "wuji_hand": sdk_module.HandModel.WujiHand,
            "wuji_hand_2": sdk_module.HandModel.WujiHand2,
        }.get(hand_model)
        if model is None:
            raise ValueError(f"unsupported hand_model: {hand_model}")
        side = {
            "left": sdk_module.Handedness.Left,
            "right": sdk_module.Handedness.Right,
        }.get(hand_side)
        if side is None:
            raise ValueError(f"unsupported hand_side: {hand_side}")
        self._session = sdk_module.RetargetSession.for_hand(model, side=side)

    def step(self, keypoints: np.ndarray) -> np.ndarray:
        """Retarget one MediaPipe-order 21x3 frame."""
        values = np.asarray(keypoints, dtype=np.float32).reshape(21, 3)
        result = np.asarray(self._session.step(values), dtype=np.float32)
        if result.shape != (20,) or not np.isfinite(result).all():
            raise RuntimeError(
                f"invalid Wuji retarget output: shape={result.shape}"
            )
        return result

    def reset(self) -> None:
        """Reset retarget warm-start and filter state."""
        self._session.reset()
