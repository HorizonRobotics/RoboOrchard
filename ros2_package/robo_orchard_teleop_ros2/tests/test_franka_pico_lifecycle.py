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

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _run_script(script: str) -> None:
    setup = """
import sys
import types
from concurrent.futures import Future
from types import SimpleNamespace

import test_pico_teleop as teleop_stubs
import test_pico_vr_node as ros_stubs

franka_msgs = types.ModuleType("franka_msgs")
franka_msgs.msg = types.ModuleType("franka_msgs.msg")
franka_msgs.msg.FrankaRobotState = type("FrankaRobotState", (), {})
sys.modules["franka_msgs"] = franka_msgs
sys.modules["franka_msgs.msg"] = franka_msgs.msg
sys.modules["xacro"] = types.ModuleType("xacro")

from robo_orchard_teleop_ros2.robot.franka import pico_vr

pico_vr.Action = teleop_stubs.Action
now = [0.0]
pico_vr.monotonic = lambda: now[0]
node = object.__new__(pico_vr.FrankaPicoVRTeleOpNode)
node._logger = ros_stubs._Logger()
node._reset_future = None
node._reset_generation = 0
node._manager_resetting = False
node._destroying = False
node._last_stamp_ns = 0
node._last_joint_warning_ns = 0
node._activation_publisher = None
node._activation_state = pico_vr.TeleopActivationState.UNAVAILABLE
node._activation_transition_id = 0
node._generated_urdf_path = None
node.reset_service_name = "/robot/control/reset"
node.current_joint_positions = [0.1] * 7
node._ee_to_flange_matrix = None
node.source_type = "right"
node.stale_timeout_s = 0.3
node._last_input_times = dict.fromkeys(("vr", "joints", "pose"), now[0])
future = Future()
requests = []

def request_reset(request):
    requests.append(request)
    return future

node.reset_client = SimpleNamespace(
    service_is_ready=lambda: True, call_async=request_reset,
)
node.teleop = teleop_stubs.VRTeleOp(
    source_type="right", urdf_path="franka.urdf",
    base_link_name="fr3_link0", ee_link_name="fr3_link8",
    trigger_intent=teleop_stubs._LongPressIntent("right"),
    reset_intent=teleop_stubs._ResetIntent("A"),
)
message = teleop_stubs._VRState(
    left_controller=teleop_stubs._Controller(
        status=1, pose=teleop_stubs._make_pose(), gripper=1.0,
    ),
    right_controller=teleop_stubs._Controller(
        status=1, pose=teleop_stubs._make_pose(), gripper=1.0,
    ),
)
node.teleop.update_robot_ee_pose(teleop_stubs._make_pose(x=1.0))
node._vr_state_callback(message)
assert node.teleop.control_state.is_active
"""
    environment = os.environ.copy()
    for name in tuple(environment):
        if name.startswith("COV_CORE_") or name == "COVERAGE_PROCESS_START":
            environment.pop(name)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(PACKAGE_ROOT), str(PACKAGE_ROOT / "tests")]
    )
    result = subprocess.run(
        [sys.executable, "-c", setup + textwrap.dedent(script)],
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("success", [True, False])
@pytest.mark.parametrize("response_order", ["before", "during", "after"])
def test_reset_waits_for_response_and_manager_then_requires_rearm(
    success, response_order
):
    _run_script(
        f"success = {success!r}\nresponse_order = {response_order!r}\n"
        + textwrap.dedent(
            """
            intent = node.teleop.control_state.reset_intent
            intent.should_reset = lambda message: True
            node._vr_state_callback(message)
            intent.should_reset = lambda message: False
            assert len(requests) == 1
            assert node._reset_future is future
            node._request_reset()
            assert len(requests) == 1
            response = SimpleNamespace(success=success, message="done")
            if response_order == "before":
                future.set_result(response)
            node._on_control_status(SimpleNamespace(data="resetting"))
            if response_order == "during":
                future.set_result(response)
            state = node.teleop.control_state
            assert state.reset_in_progress
            node._on_control_status(SimpleNamespace(data="stop"))
            if response_order == "after":
                assert state.reset_in_progress
                future.set_result(response)
            assert node._reset_future is None
            assert not state.reset_in_progress
            assert not state.is_active
            assert state.initial_vr_pose is None
            assert state.initial_ee_pose is None
            assert state.filtered_target_ee_pose is None
            assert state.rearm_required
            node._vr_state_callback(message)
            assert not state.is_active
            assert node.teleop() is None
            message.right_controller.gripper = 0.0
            node._vr_state_callback(message)
            new_pose = teleop_stubs._make_pose(x=2.0)
            node.teleop.update_robot_ee_pose(new_pose)
            message.right_controller.gripper = 1.0
            node._vr_state_callback(message)
            assert state.is_active
            assert state.initial_ee_pose is new_pose
            """
        )
    )


def test_external_reset_discards_session_without_calling_hardware():
    _run_script(
        """
        node._on_control_status(SimpleNamespace(data="resetting"))
        state = node.teleop.control_state
        assert state.reset_in_progress
        assert state.initial_vr_pose is None
        node.teleop.begin_reset = lambda: (_ for _ in ()).throw(
            AssertionError("Repeated heartbeat restarted reset")
        )
        node._on_control_status(SimpleNamespace(data="resetting"))
        node._request_reset()
        assert requests == []
        node._on_control_status(SimpleNamespace(data="stop"))
        assert not state.reset_in_progress
        assert state.rearm_required
        node._vr_state_callback(message)
        assert not state.is_active
        """
    )


@pytest.mark.parametrize("failure", ["unavailable", "request", "response"])
def test_reset_failure_never_resumes_old_session(failure):
    _run_script(
        f"failure = {failure!r}\n"
        + textwrap.dedent(
            """
            def fail_request(request):
                raise RuntimeError("service unavailable")

            if failure == "unavailable":
                node.reset_client.service_is_ready = lambda: False
            elif failure == "request":
                node.reset_client.call_async = fail_request
            node._request_reset()
            if failure == "response":
                future.set_exception(RuntimeError("reset failed"))
            assert node._reset_future is None
            state = node.teleop.control_state
            assert not state.reset_in_progress
            assert state.rearm_required
            assert not state.is_active
            node._vr_state_callback(message)
            assert not state.is_active
            """
        )
    )


def test_reset_completion_after_destroy_does_not_touch_node():
    _run_script(
        """
        node._request_reset()
        node.destroy_node()
        logs = list(node._logger.messages)
        future.set_result(SimpleNamespace(success=True, message="done"))
        assert node._reset_future is None
        assert node._logger.messages == logs
        assert node.teleop.control_state.reset_in_progress
        """
    )


def test_node_status_subscription_and_named_joint_command_contract():
    _run_script(
        """
        original_init = ros_stubs._VRTeleOp.__init__

        def teleop_init(self, **kwargs):
            original_init(self, **kwargs)
            self.ik_solver = SimpleNamespace(
                num_joints=7,
                get_joint_names=lambda: pico_vr.FRANKA_JOINT_NAMES,
            )

        ros_stubs._VRTeleOp.__init__ = teleop_init
        pico_vr.os.path.isfile = lambda path: True
        node = pico_vr.FrankaPicoVRTeleOpNode()
        assert node.reset_service_name == "/robot/control/reset"
        assert node._activation_publisher is None
        assert node._recapture_service.args[0] is pico_vr.RecaptureBaseline
        assert node._recapture_service.args[1] == "~/recapture_baseline"
        enabled = pico_vr.FrankaPicoVRTeleOpNode(parameter_overrides=[
            ros_stubs._Parameter("publish_activation", True),
        ])
        assert enabled._activation_publisher is not None
        subscription = next(
            args for args, kwargs in node.subscriptions
            if args[1] == "/robot/control/status"
        )
        assert subscription[0] is pico_vr.ControlMode
        assert subscription[2] == node._on_control_status
        assert subscription[3] == 10
        node.current_joint_positions = [0.1] * 7
        node._ee_to_flange_matrix = pico_vr.np.eye(4)
        node._robot_state_subscription = None
        node._last_input_times = dict.fromkeys(
            ("vr", "joints", "pose"), now[0]
        )
        node.teleop.next_result = SimpleNamespace(
            target_ee_pose=ros_stubs._Pose(), solution=[0.2] * 7,
        )
        node._control_callback()
        command = node.joint_command_publisher.messages[-1]
        assert isinstance(command, pico_vr.JointState)
        assert command.name == [f"fr3_joint{index}" for index in range(1, 8)]
        assert command.position == [0.2] * 7
        assert command.velocity == []
        assert command.effort == []
        node._on_control_status(SimpleNamespace(data="resetting"))
        node._control_callback()
        assert len(node.joint_command_publisher.messages) == 1
        node._manager_resetting = False
        node._reset_future = Future()
        node._control_callback()
        assert len(node.joint_command_publisher.messages) == 1
        """
    )


def _run_freshness_script(script: str) -> None:
    _run_script(
        """
node._ee_to_flange_matrix = teleop_stubs._Matrix4()
node._robot_state_subscription = None
node.base_link_name = "fr3_link0"
node.joint_command_publisher = ros_stubs._Publisher()
node.target_pose_publisher = ros_stubs._Publisher()
node.teleop.ik_solver.__class__.next_solution = [0.2] * 7
node.get_clock = lambda: SimpleNamespace(
    now=lambda: SimpleNamespace(
        nanoseconds=int(now[0] * 1e9),
        to_msg=lambda: SimpleNamespace(sec=int(now[0]), nanosec=0),
    ),
)
feedback_pose = [teleop_stubs._make_pose(x=1.0)]
joint_message = pico_vr.JointState(
    name=pico_vr.FRANKA_JOINT_NAMES, position=[0.1] * 7,
)

def refresh(exclude=None):
    if exclude != "joints":
        node._joint_state_callback(joint_message)
    if exclude != "pose":
        node._ee_pose_callback(pico_vr.PoseStamped(pose=feedback_pose[0]))
    if exclude != "vr":
        node._vr_state_callback(message)

now[0] = 0.1
refresh()
node._control_callback()
assert len(node.joint_command_publisher.messages) == 1
assert len(node.target_pose_publisher.messages) == 1
assert node.teleop.ik_solver.last_solve_joint_state == [0.1] * 7
state = node.teleop.control_state
"""
        + textwrap.dedent(script)
    )


@pytest.mark.parametrize("missing", ["vr", "joints", "pose"])
def test_stale_input_stops_output_until_fresh_release_and_reactivation(
    missing,
):
    _run_freshness_script(
        f"missing = {missing!r}\n"
        + textwrap.dedent(
            """
            message.right_controller.pose = teleop_stubs._make_pose(x=0.4)
            now[0] = 0.2
            refresh(exclude=missing)
            node._control_callback()
            before = len(node.joint_command_publisher.messages)
            for timestamp in (0.6, 1.0, 60.0):
                now[0] = timestamp
                refresh(exclude=missing)
                node._control_callback()
                assert len(node.joint_command_publisher.messages) == before
                assert len(node.target_pose_publisher.messages) == before
            assert not state.is_active
            assert state.rearm_required
            assert state.initial_vr_pose is None
            assert state.initial_ee_pose is None
            assert state.filtered_target_ee_pose is None

            feedback_pose[0] = teleop_stubs._make_pose(x=2.0)
            now[0] = 60.01
            refresh()
            node._control_callback()
            assert not state.is_active
            assert len(node.joint_command_publisher.messages) == before

            now[0] = 60.02
            message.right_controller.gripper = 0.0
            refresh()
            node._control_callback()
            assert not state.is_active
            assert len(node.joint_command_publisher.messages) == before

            now[0] = 60.03
            message.right_controller.gripper = 1.0
            refresh()
            node._control_callback()
            assert state.is_active
            assert state.initial_ee_pose.position.x == 2.0
            assert len(node.joint_command_publisher.messages) == before + 1
            command = node.joint_command_publisher.messages[-1]
            assert command.header.stamp.sec == 60
            assert command.name == pico_vr.FRANKA_JOINT_NAMES
            assert command.position == [0.2] * 7
            assert node.teleop.ik_solver.last_solve_joint_state == [0.1] * 7
            """
        )
    )


def test_input_recovery_before_timer_still_discards_the_old_session():
    _run_freshness_script(
        """
        now[0] = 60.0
        refresh()
        node._control_callback()
        assert not state.is_active
        assert state.rearm_required
        assert len(node.joint_command_publisher.messages) == 1
        """
    )


def test_configured_timeout_controls_input_expiration():
    _run_freshness_script(
        """
        node.stale_timeout_s = 1.0
        now[0] = 0.6
        node._control_callback()
        assert state.is_active
        assert len(node.joint_command_publisher.messages) == 2
        now[0] = 1.2
        node._control_callback()
        assert not state.is_active
        assert len(node.joint_command_publisher.messages) == 2
        """
    )


@pytest.mark.parametrize("missing", ["vr", "joints", "pose"])
def test_missing_input_suppresses_output(missing):
    _run_freshness_script(
        f"node._last_input_times[{missing!r}] = None\n"
        + textwrap.dedent(
            """
            node._control_callback()
            assert not state.is_active
            assert state.rearm_required
            assert len(node.joint_command_publisher.messages) == 1
            """
        )
    )


def test_invalid_controller_does_not_refresh_valid_input_age():
    _run_freshness_script(
        """
        message.right_controller.status = 0
        node._vr_state_callback(message)
        node._control_callback()
        assert node._last_input_times["vr"] is None
        assert not state.is_active
        assert state.rearm_required
        assert len(node.joint_command_publisher.messages) == 1
        message.right_controller.status = 1
        refresh()
        node._control_callback()
        assert not state.is_active
        assert len(node.joint_command_publisher.messages) == 1
        """
    )


@pytest.mark.parametrize(
    "timeout", ["0.0", "-0.1", 'float("nan")', 'float("inf")']
)
def test_stale_timeout_requires_a_finite_positive_value(timeout):
    _run_script(
        f"timeout = {timeout}\n"
        + textwrap.dedent(
            """
            pico_vr.os.path.isfile = lambda path: True
            try:
                pico_vr.FrankaPicoVRTeleOpNode(parameter_overrides=[
                    ros_stubs._Parameter("stale_timeout_s", timeout),
                ])
            except ValueError as error:
                assert "stale_timeout_s" in str(error)
            else:
                raise AssertionError("Invalid timeout was accepted")
            """
        )
    )


def test_recapture_clears_filter_reseeds_and_orders_candidates():
    _run_freshness_script(
        """
        def stamp_ns(stamp):
            return stamp.sec * 1000000000 + stamp.nanosec

        node.joint_command_publisher.topic_name = "/namespaced/remapped_arm"
        old_stamp = node.joint_command_publisher.messages[-1].header.stamp
        feedback_pose[0] = teleop_stubs._make_pose(x=2.0)
        message.right_controller.pose = teleop_stubs._make_pose(x=0.5)
        joint_message.position = [0.3] * 7
        refresh()
        response = SimpleNamespace(stamp_barrier=SimpleNamespace(
            sec=0, nanosec=0,
        ))
        node._recapture_baseline(None, response)
        assert response.success
        assert response.candidate_topic == "/namespaced/remapped_arm"
        assert stamp_ns(response.stamp_barrier) >= stamp_ns(old_stamp)
        assert state.initial_vr_pose is message.right_controller.pose
        assert state.initial_ee_pose.position.x == 2.0
        assert state.filtered_target_ee_pose is None
        assert state.current_joint_state == [0.3] * 7
        node._control_callback()
        command = node.joint_command_publisher.messages[-1]
        barrier_ns = stamp_ns(response.stamp_barrier)
        assert stamp_ns(command.header.stamp) > barrier_ns
        assert node.teleop.ik_solver.last_solve_joint_state == [0.3] * 7
        assert state.filtered_target_ee_pose.position.x == 2.0
        node._control_callback()
        next_command = node.joint_command_publisher.messages[-1]
        assert stamp_ns(next_command.header.stamp) > stamp_ns(
            command.header.stamp,
        )
        """
    )


@pytest.mark.parametrize("inactive", ["released", "rearm"])
def test_recapture_does_not_activate_or_clear_rearm(inactive):
    _run_freshness_script(
        f"inactive = {inactive!r}\n"
        + textwrap.dedent(
            """
            node.teleop.reset_session(require_rearm=inactive == "rearm")
            node.joint_command_publisher.topic_name = "/candidate"
            response = SimpleNamespace(stamp_barrier=SimpleNamespace(
                sec=0, nanosec=0,
            ))
            node._recapture_baseline(None, response)
            assert response.success
            assert not state.is_active
            assert state.rearm_required == (inactive == "rearm")
            node._control_callback()
            assert len(node.joint_command_publisher.messages) == 1
            """
        )
    )


@pytest.mark.parametrize(
    "failure", ["vr", "joints", "pose", "reset", "capture"]
)
def test_recapture_failure_has_no_cutover_or_session_restore(failure):
    _run_freshness_script(
        f"failure = {failure!r}\n"
        + textwrap.dedent(
            """
            if failure == "reset":
                node._on_control_status(SimpleNamespace(data="resetting"))
            elif failure == "capture":
                node.teleop.recapture_baseline = lambda: False
            else:
                node._last_input_times[failure] = None
            response = SimpleNamespace(stamp_barrier=SimpleNamespace(
                sec=99, nanosec=99,
            ))
            node._recapture_baseline(None, response)
            assert not response.success
            assert response.reason
            assert response.candidate_topic == ""
            assert response.stamp_barrier.sec == 0
            assert response.stamp_barrier.nanosec == 0
            assert not state.is_active
            assert state.rearm_required
            node._control_callback()
            assert len(node.joint_command_publisher.messages) == 1
            """
        )
    )


def test_activation_heartbeat_distinguishes_release_pending_and_fault():
    _run_freshness_script(
        """
        node._activation_publisher = ros_stubs._Publisher()
        activation = pico_vr.TeleopActivationState
        def heartbeat():
            node._control_callback()
            return node._activation_publisher.messages[-1]

        assert heartbeat().state == activation.ACTIVE
        previous = heartbeat().transition_id
        assert heartbeat().transition_id == previous
        node.teleop.reset_session(require_rearm=True)
        assert heartbeat().state == activation.UNAVAILABLE
        message.right_controller.gripper = 0.0
        refresh()
        assert heartbeat().state == activation.INACTIVE
        message.right_controller.gripper = 1.0
        state.trigger_intent.is_active = lambda message: False
        refresh()
        assert heartbeat().state == activation.PENDING
        state.trigger_intent.is_active = lambda message: True
        refresh()
        assert heartbeat().state == activation.ACTIVE
        now[0] = 1.0
        assert heartbeat().state == activation.UNAVAILABLE
        refresh()
        assert heartbeat().state == activation.UNAVAILABLE
        """
    )


def test_startup_held_grip_never_reports_release():
    _run_freshness_script(
        """
        node._activation_publisher = ros_stubs._Publisher()
        node.teleop.reset_session()
        state.trigger_intent.is_active = lambda message: False
        node._control_callback()
        assert node._activation_publisher.messages[-1].state \
            == pico_vr.TeleopActivationState.PENDING
        state.trigger_intent.is_active = lambda message: True
        refresh()
        node._control_callback()
        assert node._activation_publisher.messages[-1].state \
            == pico_vr.TeleopActivationState.ACTIVE
        assert all(item.state != pico_vr.TeleopActivationState.INACTIVE
                   for item in node._activation_publisher.messages)
        """
    )


@pytest.mark.parametrize("invalid", ["vr_pose", "grip", "joints", "pose"])
def test_invalid_numeric_input_invalidates_session_and_recapture(invalid):
    _run_freshness_script(
        f"invalid = {invalid!r}\n"
        + textwrap.dedent(
            """
            if invalid == "vr_pose":
                message.right_controller.pose.orientation.w = 0.0
                node._vr_state_callback(message)
            elif invalid == "grip":
                message.right_controller.gripper = float("nan")
                node._vr_state_callback(message)
            elif invalid == "joints":
                joint_message.position[0] = float("nan")
                node._joint_state_callback(joint_message)
            else:
                feedback_pose[0].position.x = float("nan")
                node._ee_pose_callback(pico_vr.PoseStamped(
                    pose=feedback_pose[0],
                ))
            response = SimpleNamespace(stamp_barrier=SimpleNamespace(
                sec=0, nanosec=0,
            ))
            node._recapture_baseline(None, response)
            assert not response.success
            assert not state.is_active
            assert state.rearm_required
            """
        )
    )
