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

import importlib
import sys
import types

import pytest


def _install_stub_modules():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.PoseStamped = type("PoseStamped", (), {})
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs.msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = type("JointState", (), {})
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs.msg

    piper_sdk = types.ModuleType("piper_sdk")
    piper_sdk.C_PiperInterface = type("C_PiperInterface", (), {})
    sys.modules["piper_sdk"] = piper_sdk

    piper_msg = types.ModuleType("robo_orchard_piper_msg_ros2")
    piper_msg.msg = types.ModuleType("robo_orchard_piper_msg_ros2.msg")
    piper_msg.msg.PiperStatusMsg = type("PiperStatusMsg", (), {})
    sys.modules["robo_orchard_piper_msg_ros2"] = piper_msg
    sys.modules["robo_orchard_piper_msg_ros2.msg"] = piper_msg.msg

    scipy = types.ModuleType("scipy")
    scipy.spatial = types.ModuleType("scipy.spatial")
    scipy.spatial.transform = types.ModuleType("scipy.spatial.transform")

    class FakeRotation:
        @staticmethod
        def from_euler(*args, **kwargs):
            return types.SimpleNamespace(as_quat=lambda: [0.0, 0.0, 0.0, 1.0])

    scipy.spatial.transform.Rotation = FakeRotation
    sys.modules["scipy"] = scipy
    sys.modules["scipy.spatial"] = scipy.spatial
    sys.modules["scipy.spatial.transform"] = scipy.spatial.transform


_install_stub_modules()

sys.path.insert(0, "ros2_package/robo_orchard_piper_ros2")
sys.modules.pop("robo_orchard_piper_ros2.ros_bridge", None)

ros_bridge = importlib.import_module("robo_orchard_piper_ros2.ros_bridge")

JOINT_NAME_LAYOUTS = [
    list(ros_bridge.DEFAULT_JOINT_NAMES),
    [f"left_joint{index}" for index in range(1, 7)] + ["left_gripper"],
    ["shoulder", "upper", "elbow", "forearm", "wrist", "tool", "opening"],
    ["joint6", "joint5", "joint4", "joint3", "joint2", "joint1", "joint7"],
]


def _feedback_piper():
    joints = types.SimpleNamespace(
        **{f"joint_{index}": index * 1000 for index in range(1, 7)}
    )
    gripper = types.SimpleNamespace(grippers_angle=25000, grippers_effort=1000)
    motors = types.SimpleNamespace(
        **{
            f"motor_{index}": types.SimpleNamespace(
                motor_speed=index * 1000, effort=index * 1000
            )
            for index in range(1, 7)
        }
    )
    return types.SimpleNamespace(
        GetArmJointMsgs=lambda: types.SimpleNamespace(joint_state=joints),
        GetArmJointCtrl=lambda: types.SimpleNamespace(joint_ctrl=joints),
        GetArmHighSpdInfoMsgs=lambda: motors,
        GetArmGripperMsgs=lambda: types.SimpleNamespace(gripper_state=gripper),
        GetArmGripperCtrl=lambda: types.SimpleNamespace(gripper_ctrl=gripper),
    )


@pytest.mark.parametrize("names", JOINT_NAME_LAYOUTS)
@pytest.mark.parametrize("getter", ["get_arm_state", "get_arm_ctrl_state"])
def test_feedback_uses_configured_names_without_changing_values(names, getter):
    piper = _feedback_piper()
    original = getattr(ros_bridge, getter)(piper)
    renamed = getattr(ros_bridge, getter)(piper, names)

    assert renamed.name == names
    assert renamed.name is not names
    assert original.name == [f"joint{index}" for index in range(1, 7)] + [
        "gripper"
    ]
    assert renamed.position == original.position
    assert renamed.velocity == original.velocity
    assert renamed.effort == original.effort


def _command_piper():
    calls = []
    return types.SimpleNamespace(
        JointCtrl=lambda *args: calls.append(("arm", args)),
        GripperCtrl=lambda *args: calls.append(("gripper", args)),
    ), calls


@pytest.mark.parametrize("names", JOINT_NAME_LAYOUTS)
@pytest.mark.parametrize("order", [list(range(7)), [6, 2, 0, 5, 1, 4, 3]])
def test_command_uses_configured_names_including_reordered_gripper(
    names, order
):
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.025]
    message = types.SimpleNamespace(
        name=[names[index] for index in order],
        position=[positions[index] for index in order],
    )
    piper, calls = _command_piper()

    ros_bridge.joint_control(piper, message, joint_names=names)

    assert calls == [
        ("arm", tuple(round(value * 57324.840764) for value in positions[:6])),
        ("gripper", (25000, 1000, 0x01, 0)),
    ]


@pytest.mark.parametrize(
    "invalid",
    [
        "missing",
        "duplicate",
        "foreign_names",
        "length",
        "nan",
        "inf",
        "joint7",
        "large_arm",
        "large_gripper",
    ],
)
def test_invalid_commands_never_call_sdk(invalid):
    expected_names = list(JOINT_NAME_LAYOUTS[2])
    names = list(expected_names)
    positions = [0.1] * 7
    if invalid == "missing":
        names.pop()
        positions.pop()
    elif invalid == "duplicate":
        names[1] = names[0]
    elif invalid == "foreign_names":
        names = list(JOINT_NAME_LAYOUTS[1])
    elif invalid == "length":
        positions.pop()
    elif invalid in {"nan", "inf"}:
        positions[-1] = float(invalid)
    elif invalid == "joint7":
        names[-1] = "left_joint7"
    elif invalid == "large_arm":
        positions[0] = 1e308
    elif invalid == "large_gripper":
        positions[-1] = 1e308
    piper, calls = _command_piper()

    with pytest.raises(ValueError):
        ros_bridge.joint_control(
            piper,
            types.SimpleNamespace(name=names, position=positions),
            joint_names=expected_names,
        )

    assert calls == []


@pytest.mark.parametrize("include_gripper", [False, True])
def test_disabled_gripper_does_not_require_or_send_a_gripper_target(
    include_gripper,
):
    expected_names = list(JOINT_NAME_LAYOUTS[2])
    names = list(expected_names)
    if not include_gripper:
        names = names[:6]
    piper, calls = _command_piper()

    ros_bridge.joint_control(
        piper,
        types.SimpleNamespace(name=names, position=[0.1] * len(names)),
        has_gripper=False,
        joint_names=expected_names,
    )

    assert len(calls) == 1
    assert calls[0][0] == "arm"


@pytest.mark.parametrize(
    "names",
    [
        None,
        "abcdefg",
        [],
        ["name"] * 7,
        [f"name{index}" for index in range(6)],
        [f"name{index}" for index in range(8)],
        ["", "b", "c", "d", "e", "f", "g"],
        [" ", "b", "c", "d", "e", "f", "g"],
        [1, "b", "c", "d", "e", "f", "g"],
    ],
)
def test_invalid_configured_names_fail_before_sdk_access(names):
    piper, calls = _command_piper()
    with pytest.raises(ValueError, match="joint_names"):
        ros_bridge.validate_joint_names(names)
    with pytest.raises(ValueError, match="joint_names"):
        ros_bridge.get_arm_state(piper, names)
    with pytest.raises(ValueError, match="joint_names"):
        ros_bridge.get_arm_ctrl_state(piper, names)
    with pytest.raises(ValueError, match="joint_names"):
        ros_bridge.joint_control(
            piper,
            types.SimpleNamespace(name=[], position=[]),
            joint_names=names,
        )
    assert calls == []


def test_switch_piper_ctrl_mode_retries_until_target_mode_is_observed(
    monkeypatch,
):
    arm_status = types.SimpleNamespace(ctrl_mode=0x02)
    motion_calls = []
    clock = {"now": 0.0}

    def fake_motion(*args, **kwargs):
        motion_calls.append((args, kwargs))
        if len(motion_calls) == 3:
            arm_status.ctrl_mode = 0x01

    piper = types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(arm_status=arm_status),
        MotionCtrl_2=fake_motion,
    )

    monkeypatch.setattr(ros_bridge.time, "time", lambda: clock["now"])
    monkeypatch.setattr(
        ros_bridge.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    ros_bridge.switch_piper_ctrl_mode(piper, 0x01, is_mit=True, timeout=5.0)
    assert len(motion_calls) == 3
    assert all(kwargs == {"is_mit_mode": 0xAD} for _, kwargs in motion_calls)


def test_switch_piper_ctrl_mode_times_out_when_target_mode_stays_unmatched(
    monkeypatch,
):
    arm_status = types.SimpleNamespace(ctrl_mode=0x02)
    motion_calls = []
    clock = {"now": 0.0}

    piper = types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(arm_status=arm_status),
        MotionCtrl_2=lambda *args, **kwargs: motion_calls.append(
            (args, kwargs)
        ),
    )

    monkeypatch.setattr(ros_bridge.time, "time", lambda: clock["now"])
    monkeypatch.setattr(
        ros_bridge.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    with pytest.raises(TimeoutError):
        ros_bridge.switch_piper_ctrl_mode(
            piper, 0x01, is_mit=False, timeout=0.5
        )

    assert motion_calls
    assert all(kwargs == {"is_mit_mode": 0x00} for _, kwargs in motion_calls)
