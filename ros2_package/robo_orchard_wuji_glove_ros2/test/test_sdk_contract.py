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

from pathlib import Path

import numpy as np
import pytest

from robo_orchard_wuji_glove_ros2.sdk_client import (
    SDK_STREAM_SPECS,
    SUPPORTED_WUJI_SDK_VERSION,
    WujiSdkRetargeter,
    load_wuji_sdk,
)


def test_installed_sdk_and_declared_pins_match() -> None:
    """Check the real wheel and all three version declarations."""
    _, version = load_wuji_sdk()
    assert version == "2026.10.26"
    package = Path(__file__).resolve().parents[1]
    pin = f"wuji-sdk=={SUPPORTED_WUJI_SDK_VERSION}"
    assert f'"{pin}"' in (package / "setup.py").read_text()
    assert (
        pin in (package.parent / "requirements.txt").read_text().splitlines()
    )


def test_subscription_and_connection_api_matches_real_wheel() -> None:
    """Inspect native descriptors without SDK initialization or connection."""
    sdk, _ = load_wuji_sdk()
    assert callable(sdk.Subscription.set_rate)
    assert "set_rate" not in dir(sdk.CallbackSubscription)
    assert "emf_poses_rate_divider" not in dir(sdk.WujiGlove)
    assert callable(sdk.EmfPosesResource.subscribe)
    assert callable(sdk.EmfPosesResource.subscribe_with_callback)
    for spec in SDK_STREAM_SPECS:
        assert callable(getattr(sdk.WujiGlove, spec.name))
    for method in (
        "current_user",
        "list_users",
        "switch_user",
        "switch_to_default_user",
        "connect",
        "disconnect",
        "tf",
        "tf_static",
    ):
        assert callable(getattr(sdk.SdkManager, method))
    for method in ("sync_time", "hand_model_path", "hand_side"):
        assert callable(getattr(sdk.WujiGlove, method))
    options = sdk.ConnectOptions(timeout_ms=1_000, retry_count=3)
    assert options.auto_time_sync_interval_ms == 30_000


@pytest.mark.parametrize(
    ("type_name", "fields"),
    [
        ("TimeSyncResult", {"offset_us", "round_trip_us", "synced_at_us"}),
        ("FrameHeader", {"seq", "timestamp_us", "frame_id"}),
        ("Pose", {"position", "orientation"}),
        ("EmfPoseArray", {"header", "poses"}),
        ("HandJointAngles", {"header", "fingers"}),
        ("FingerJointAngles", {"angles", "confidence"}),
        ("HandSkeleton", {"header", "joints"}),
        ("SkeletonJoint", {"name", "pose", "confidence"}),
        (
            "ImuData",
            {
                "header",
                "orientation",
                "orientation_covariance",
                "angular_velocity",
                "angular_velocity_covariance",
                "linear_acceleration",
                "linear_acceleration_covariance",
            },
        ),
        (
            "PointCloud",
            {"header", "frame_id", "point_stride", "fields", "data"},
        ),
        ("TactileFrame", {"header", "data"}),
        (
            "TactileZones",
            {"header", "palm", "thumb", "index", "middle", "ring", "pinky"},
        ),
        ("FrameTransforms", {"transforms"}),
    ],
)
def test_stream_fields_match_real_sdk(
    type_name: str, fields: set[str]
) -> None:
    """Verify the adapter's field access against actual extension classes."""
    sdk, _ = load_wuji_sdk()
    assert fields <= set(dir(getattr(sdk, type_name)))


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("model", ["wuji_hand", "wuji_hand_2"])
def test_real_sdk_retarget_is_offline(model: str, side: str) -> None:
    """Retarget synthetic 21-point input without SdkManager or hardware."""
    keypoints = np.zeros((21, 3), dtype=np.float32)
    for finger in range(5):
        for joint in range(4):
            keypoints[1 + 4 * finger + joint] = (
                (finger - 2) * 0.02,
                (joint + 1) * 0.025,
                0.0,
            )
    retargeter = WujiSdkRetargeter(model, side)
    for _ in range(2):
        result = retargeter.step(keypoints)
        assert result.shape == (20,)
        assert result.dtype == np.float32
        assert np.isfinite(result).all()
        retargeter.reset()
