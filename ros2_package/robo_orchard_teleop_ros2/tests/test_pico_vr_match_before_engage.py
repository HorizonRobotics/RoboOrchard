# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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
import sys
import types

import test_pico_vr_node as pico_stubs

import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module
from robo_orchard_teleop_ros2.bridge.pico.teleop import Action, VRTeleOp
from robo_orchard_teleop_ros2.robot.piper.pico_vr import (
    JOINT_NAMES,
    ArmEngageState,
    PiperPicoVRTeleOpNode,
)

JointState = sys.modules["sensor_msgs.msg"].JointState
Pose = sys.modules["geometry_msgs.msg"].Pose
Trigger = sys.modules["std_srvs.srv"].Trigger
VRState = sys.modules["robo_orchard_pico_msg_ros2.msg"].VRState


def _patch_vr_teleop_stub():
    if getattr(VRTeleOp, "_match_test_patched", False):
        return

    original_call = VRTeleOp.__call__

    def _counting_call(self):
        self.call_count = getattr(self, "call_count", 0) + 1
        return original_call(self)

    def _recapture_baseline(self):
        self.recapture_count = getattr(self, "recapture_count", 0) + 1
        return getattr(self, "recapture_result", True)

    VRTeleOp.__call__ = _counting_call
    VRTeleOp.recapture_baseline = _recapture_baseline
    VRTeleOp._match_test_patched = True


_patch_vr_teleop_stub()


class _FakeFuture:
    def __init__(self, response=None, *, done: bool = True):
        self._response = response
        self._done = done
        self._callbacks = []

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
            return
        self._callbacks.append(callback)

    def result(self):
        return self._response

    def set_result(self, response):
        self._response = response
        self._done = True
        callbacks = list(self._callbacks)
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class _FakeClient:
    def __init__(self, response=None, *, ready: bool = True):
        self.response = response
        self.ready = ready
        self.calls = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        future = _FakeFuture(self.response, done=self.response is not None)
        self.futures.append(future)
        return future


def _trigger_response(success: bool = True, message: str = "ok"):
    response = Trigger.Response()
    response.success = success
    response.message = message
    return response


def _joint_state(gripper: float):
    return JointState(
        name=JOINT_NAMES,
        position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripper],
        velocity=[0.0] * 7,
        effort=[0.0] * 7,
    )


def _vr_state(*, left_trigger: float = 0.0, right_trigger: float = 0.0):
    left_controller = types.SimpleNamespace(
        status=1,
        trigger=left_trigger,
        gripper=0.0,
        pose=object(),
    )
    right_controller = types.SimpleNamespace(
        status=1,
        trigger=right_trigger,
        gripper=0.0,
        pose=object(),
    )
    try:
        msg = VRState(
            left_controller=left_controller,
            right_controller=right_controller,
        )
    except TypeError:
        msg = VRState()
        msg.left_controller = left_controller
        msg.right_controller = right_controller
    return msg


def _build_node(*, match_tolerance: float = 0.1):
    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode(
        parameter_overrides=[
            pico_stubs._Parameter("urdf_path", "/tmp/robot.urdf"),
            pico_stubs._Parameter("match_tolerance", match_tolerance),
        ]
    )
    node._left_reset_service = "/robot/left/reset_ctrl"
    node._right_reset_service = "/robot/right/reset_ctrl"
    node._left_reset_client = _FakeClient()
    node._right_reset_client = _FakeClient()
    return node


def _messages(node, level: str):
    return [
        msg
        for logged_level, msg in node._logger.messages
        if logged_level == level
    ]


def test_deactive_to_waiting_on_active_edge():
    node = _build_node()
    node.left_joint_state_msg = _joint_state(0.025)

    node._on_vr_state_side("left", Action.ACTIVE, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.WAITING_FOR_MATCH
    assert any(
        "waiting for trigger to match" in msg
        for msg in _messages(node, "info")
    )


def test_deactive_stays_if_no_joint_state():
    node = _build_node()

    node._on_vr_state_side("left", Action.ACTIVE, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.DEACTIVE
    assert any(
        "joint_state not yet received" in msg
        for msg in _messages(node, "warn")
    )


def test_waiting_to_active_when_match_satisfied():
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.WAITING_FOR_MATCH
    node.left_joint_state_msg = _joint_state(0.025)

    node._on_vr_state_side(
        "left",
        Action.ACTIVE,
        _vr_state(left_trigger=0.75),
    )

    assert node._arm_state["left"] == ArmEngageState.ACTIVE
    assert VRTeleOp.instances[0].recapture_count == 1
    assert any("VR teleop is ACTIVE" in msg for msg in _messages(node, "info"))


def test_waiting_stays_if_delta_too_large():
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.WAITING_FOR_MATCH
    node.left_joint_state_msg = _joint_state(0.1)

    node._on_vr_state_side(
        "left",
        Action.ACTIVE,
        _vr_state(left_trigger=1.0),
    )

    assert node._arm_state["left"] == ArmEngageState.WAITING_FOR_MATCH


def test_release_before_match_cancels_local_engage():
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.WAITING_FOR_MATCH

    node._on_vr_state_side("left", Action.DEACTIVE, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.DEACTIVE


def test_active_release_only_pauses_vr_output():
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.ACTIVE

    node._on_vr_state_side("left", Action.DEACTIVE, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.DEACTIVE
    assert any(
        "DAgger mode is unchanged" in msg for msg in _messages(node, "info")
    )


def test_timer_suppresses_publish_during_waiting():
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.WAITING_FOR_MATCH
    left_teleop = VRTeleOp.instances[0]
    left_teleop.next_result = pico_stubs._TeleOpResult(
        target_ee_pose=Pose(),
        solution=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )

    node.timer_callback()

    assert getattr(left_teleop, "call_count", 0) == 0
    assert node.left_cmd_pub.messages == []
    assert node.left_target_pub.messages == []


def test_left_and_right_state_are_independent():
    node = _build_node()
    node.left_joint_state_msg = _joint_state(0.025)

    node._on_vr_state_side("left", Action.ACTIVE, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.WAITING_FOR_MATCH
    assert node._arm_state["right"] == ArmEngageState.DEACTIVE


# ---------------------------------------------------------------------------
# RESET chain: state-aware reset_ctrl dispatch
# ---------------------------------------------------------------------------


def test_reset_ignored_in_deactive():
    # RESET only makes sense once the operator has expressed engage intent
    # (gripper held). In DEACTIVE we log and do nothing -- no service calls.
    node = _build_node()
    node._left_reset_client.response = _trigger_response(success=True)

    node._on_vr_state_side("left", Action.RESET, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.DEACTIVE
    assert node._left_reset_client.calls == []
    assert any(
        "engage gripper first" in msg.lower()
        for msg in _messages(node, "info")
    )


def test_reset_refused_when_no_reset_service_configured():
    # If reset_service is empty, RESET is inert; engage path is unaffected.
    node = _build_node()
    node._left_reset_service = ""
    node._left_reset_client = None
    node._arm_state["left"] = ArmEngageState.ACTIVE

    node._on_vr_state_side("left", Action.RESET, _vr_state())

    # State unchanged -- ACTIVE, no chain kicked off.
    assert node._arm_state["left"] == ArmEngageState.ACTIVE
    assert any(
        "no reset_service configured" in msg.lower()
        for msg in _messages(node, "error")
    )


def test_reset_idempotent_in_arm_resetting():
    # A second RESET while a chain is already running must be a no-op.
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.ARM_RESETTING

    node._on_vr_state_side("left", Action.RESET, _vr_state())

    assert node._arm_state["left"] == ArmEngageState.ARM_RESETTING
    assert node._left_reset_client.calls == []


def test_reset_from_active_calls_reset_ctrl_only():
    # ACTIVE -> ARM_RESETTING -> reset_ctrl -> DEACTIVE.
    # DAgger mode is controlled by the frontend and is not changed here.
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.ACTIVE
    node._left_reset_client.response = _trigger_response(success=True)

    node._on_vr_state_side("left", Action.RESET, _vr_state())

    assert len(node._left_reset_client.calls) == 1
    assert node._arm_state["left"] == ArmEngageState.DEACTIVE
    assert any(
        "DAgger mode unchanged" in msg for msg in _messages(node, "info")
    )


def test_reset_from_waiting_calls_reset_ctrl_only():
    # WAITING_FOR_MATCH -> ARM_RESETTING -> reset_ctrl -> DEACTIVE.
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.WAITING_FOR_MATCH
    node._left_reset_client.response = _trigger_response(success=True)

    node._on_vr_state_side("left", Action.RESET, _vr_state())

    assert len(node._left_reset_client.calls) == 1
    assert node._arm_state["left"] == ArmEngageState.DEACTIVE


def test_reset_failure_returns_deactive_without_mode_switch():
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.ACTIVE
    node._left_reset_client.response = _trigger_response(
        success=False, message="busy"
    )

    node._on_vr_state_side("left", Action.RESET, _vr_state())

    assert len(node._left_reset_client.calls) == 1
    assert node._arm_state["left"] == ArmEngageState.DEACTIVE


def test_timer_suppresses_publish_during_arm_resetting():
    # While the reset chain runs, the VR-rate timer must not push joint
    # cmds (they would either be dropped by the muxer or race the SDK
    # path inside reset_ctrl).
    node = _build_node()
    node._arm_state["left"] = ArmEngageState.ARM_RESETTING
    left_teleop = VRTeleOp.instances[0]
    left_teleop.next_result = pico_stubs._TeleOpResult(
        target_ee_pose=Pose(),
        solution=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )

    node.timer_callback()

    assert getattr(left_teleop, "call_count", 0) == 0
    assert node.left_cmd_pub.messages == []
    assert node.left_target_pub.messages == []
