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

"""Pose encoding and request-bound reference times without a ROS install."""

import copy
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from test_action_exec import _deploy_config, _FakeNode

from robo_orchard_deploy_ros2.action_exec import ActionExecutor
from robo_orchard_deploy_ros2.config import (
    JointCommandChannel,
    PoseCommandChannel,
    TrajectoryStitchConfig,
)
from robo_orchard_deploy_ros2.node import async_node, sync_node
from robo_orchard_deploy_ros2.obs_manager import ObservationSnapshot
from robo_orchard_deploy_ros2.trajectory_stitcher import TrajectoryStitcher

POSE = [1, 2, 3, 2, 0, 0, 2]


def _channels(joint=True):
    return (
        [
            JointCommandChannel(
                server_output_key="arm",
                topic="/arm_cmd",
                joint_names=["joint"],
                server_remaining_key="tail",
            )
        ]
        if joint
        else []
    ) + [
        PoseCommandChannel(
            server_output_key="base", topic="/base_cmd", frame_id="base_link"
        )
    ]


def _stamp(message):
    return message.header.stamp.sec, message.header.stamp.nanosec


@pytest.mark.parametrize("scale", [1, -1, 1e-300, 1e300])
def test_pose_units_quaternion_order_normalization_and_raw_ownership(scale):
    node = _FakeNode()
    executor = ActionExecutor(node, _deploy_config(_channels()))
    action = {"arm": [[0.4]], "base": [POSE.copy()]}
    action["base"][0][3:] = [x * scale for x in POSE[3:]]
    original = copy.deepcopy(action)
    executor.send_action(action, 0, (123, 456))
    message = node.publishers_by_topic["/base_cmd"].published[-1]
    assert _stamp(message) == (123, 456)
    assert message.header.frame_id == "base_link"
    assert len(message.poses) == 1
    assert message.poses[0].header.frame_id == "base_link"
    assert _stamp(message.poses[0]) == (123, 40_000_456)
    assert vars(message.poses[0].pose.position) == dict(x=1, y=2, z=3)
    q = message.poses[0].pose.orientation
    assert [q.x, q.y, q.z, q.w] == pytest.approx(
        [0, 0, np.sign(scale) / np.sqrt(2), np.sign(scale) / np.sqrt(2)]
    )
    assert action == original


def test_missing_pose_reference_skips_pose_publication():
    node = _FakeNode()
    executor = ActionExecutor(node, _deploy_config(_channels()))
    executor.send_action({"arm": [[0.4]], "base": [POSE]}, 0)
    assert not node.publishers_by_topic["/base_cmd"].published
    assert node.publishers_by_topic["/arm_cmd"].published[-1].position == [0.4]


def test_pose_does_not_enter_joint_limiter_or_rtc_tails():
    node = _FakeNode()
    executor = ActionExecutor(
        node, _deploy_config(_channels(), max_command_velocity=1)
    )
    first = {"arm": [[0]], "base": [POSE]}
    second = {"arm": [[10], [20]], "base": [POSE, POSE]}
    executor.send_action(first, 0, (1, 0))
    executor.send_action(second, 0, (2, 0))
    assert node.publishers_by_topic["/arm_cmd"].published[-1].position == [
        0.04
    ]
    assert (
        node.publishers_by_topic["/base_cmd"]
        .published[-1]
        .poses[0]
        .pose.position.x
        == 1
    )
    assert set(executor._limiter._last) == {"arm"}
    tails = executor.remaining_actions(second, 0)
    assert set(tails) == {"tail"}
    np.testing.assert_array_equal(tails["tail"], [[20]])
    executor.reset_limiter()
    assert len(node.publishers_by_topic["/base_cmd"].published) == 2


def _runtime(module, *, joint=False):
    node = module.DeployNode.__new__(module.DeployNode)
    transport = _FakeNode()
    node.get_logger = lambda: Mock()
    node.get_clock = transport.get_clock
    node.create_publisher = transport.create_publisher
    node.publishers = transport.publishers_by_topic
    node.config = _deploy_config(_channels(joint))
    node.config.control_config.control_frequency = 200.0
    node.action_executor = ActionExecutor(node, node.config)
    node.current_actions = None
    node._action_observation_stamp = None
    node._inference_generation = 0
    node._status_publisher = Mock()
    node._event_publisher = Mock()
    node.snapshot = ObservationSnapshot({"image": 1}, (11, 12))
    node.obs_manager = SimpleNamespace(get_observations=lambda: node.snapshot)
    node.model_inferencer = SimpleNamespace(request_inference=lambda obs: None)
    if module is sync_node:
        node.state_lock = threading.Lock()
        node.state = module.NodeState.IDLE
        node.current_action_index = 0
    else:
        node.shared_state_lock = threading.Lock()
        node.state = module.NodeState.EXECUTING
        node.current_action_idx = 0
        node._chunk_start_idx = 0
        node._pending_actions = None
        node._stitch_lead_steps = 1
        node.max_delay_horizon = 100
        node._stitcher = TrajectoryStitcher(None, 200, [])
    return node


def _request(node, module, action):
    node.model_inferencer.request_inference = lambda obs: action
    _infer(node, module)


def _infer(node, module):
    if module is sync_node:
        node._model_infer_callback()
    else:
        node._model_infer_timer_callback()


@pytest.mark.parametrize("module", [sync_node, async_node])
@pytest.mark.parametrize("not_a_chunk", [1.0, "x", {"x": POSE}])
def test_inference_rejects_pose_output_that_is_not_a_list(module, not_a_chunk):
    node = _runtime(module, joint=True)
    _request(node, module, {"base": not_a_chunk, "arm": [[0.1]]})
    assert node.current_actions is None

    _request(node, module, {"base": [POSE], "arm": [[0.2]]})
    node._action_timer_callback()
    assert node.publishers["/arm_cmd"].published[-1].position == [0.2]
    assert len(node.publishers["/base_cmd"].published) == 1


@pytest.mark.parametrize("module", [sync_node, async_node])
@pytest.mark.parametrize(
    "bad_pose",
    [[0] * 6, [0] * 7, [0, 0, 0, 1, 0, 0, float("nan")], ["bad"] * 7],
)
def test_bad_middle_pose_rejects_entire_mixed_chunk(module, bad_pose):
    node = _runtime(module, joint=True)
    _request(
        node,
        module,
        {"arm": [[0.1], [0.2], [0.3]], "base": [POSE, bad_pose, POSE]},
    )
    assert node.current_actions is None
    node._action_timer_callback()
    assert not node.publishers["/arm_cmd"].published
    assert not node.publishers["/base_cmd"].published

    _request(node, module, {"arm": [[0.4]], "base": [POSE]})
    node._action_timer_callback()
    assert node.publishers["/arm_cmd"].published[-1].position == [0.4]
    assert len(node.publishers["/base_cmd"].published) == 1


def test_async_bad_pose_chunk_does_not_replace_live_chunk():
    node = _runtime(async_node, joint=True)
    live = {"arm": [[0.1], [0.2]], "base": [POSE, POSE]}
    _request(node, async_node, live)
    node._action_timer_callback()
    node.snapshot = ObservationSnapshot({"image": 2}, (22, 23))

    _request(
        node, async_node, {"arm": [[0.5], [0.6]], "base": [POSE, [0] * 7]}
    )
    node._action_timer_callback()
    assert node.current_actions == live
    assert node.publishers["/arm_cmd"].published[-1].position == [0.2]
    assert _stamp(node.publishers["/base_cmd"].published[-1]) == (11, 12)


@pytest.mark.parametrize("module", [sync_node, async_node])
def test_request_reference_survives_new_observations_and_exhaustion(module):
    node = _runtime(module)

    def request(data):
        assert data == {"image": 1}
        node.snapshot = ObservationSnapshot({"image": 2}, (99, 99))
        return {"base": [POSE, POSE]}

    node.model_inferencer.request_inference = request
    _infer(node, module)
    for _ in range(4):
        node._action_timer_callback()
    messages = node.publishers["/base_cmd"].published
    assert len(messages) == 2
    assert [_stamp(msg) for msg in messages] == [(11, 12)] * 2
    assert [_stamp(msg.poses[0]) for msg in messages] == [
        (11, 5_000_012),
        (11, 10_000_012),
    ]
    node._disable_inference_callback(None, SimpleNamespace())
    node._enable_inference_callback(None, SimpleNamespace())
    node._action_timer_callback()
    assert len(messages) == 2  # No invented stop or replayed pose.
    _request(node, module, {"base": [POSE]})
    node._action_timer_callback()
    assert _stamp(messages[-1]) == (99, 99)


@pytest.mark.parametrize("module", [sync_node, async_node])
def test_stale_response_cannot_replace_reference_after_disable_resume(module):
    node = _runtime(module)

    def request(data):
        node._disable_inference_callback(None, SimpleNamespace())
        node._enable_inference_callback(None, SimpleNamespace())
        node.snapshot = ObservationSnapshot({"image": 2}, (22, 23))
        return {"base": [POSE]}

    node.model_inferencer.request_inference = request
    _infer(node, module)
    assert node.current_actions is None
    assert node._action_observation_stamp is None
    _request(node, module, {"base": [POSE]})
    node._action_timer_callback()
    assert _stamp(node.publishers["/base_cmd"].published[-1]) == (22, 23)


def test_async_stale_response_cannot_overwrite_a_replaced_chunk_reference():
    node = _runtime(async_node)
    _request(node, async_node, {"base": [POSE]})
    node.snapshot = ObservationSnapshot({"image": 2}, (22, 23))

    def request(data):
        # A different valid handover wins while this request is in flight.
        node._switch_actions({"base": [POSE]}, 0, (33, 34))
        return {"base": [POSE]}

    node.model_inferencer.request_inference = request
    _infer(node, async_node)
    node._action_timer_callback()
    assert _stamp(node.publishers["/base_cmd"].published[-1]) == (33, 34)


@pytest.mark.parametrize("module", [sync_node, async_node])
def test_publication_failure_invalidates_reference_without_a_stop_pose(module):
    node = _runtime(module)
    _request(node, module, {"base": [POSE]})
    publisher = node.publishers["/base_cmd"]
    publisher.publish = Mock(side_effect=RuntimeError("transport failure"))
    node._action_timer_callback()
    assert node.current_actions is None
    assert node._action_observation_stamp is None
    assert publisher.publish.call_count == 1
    node._action_timer_callback()
    assert publisher.publish.call_count == 1


def test_async_rtc_skips_rows_without_changing_request_reference():
    node = _runtime(async_node, joint=True)
    action = {"arm": [[0.1]] * 5, "base": [POSE] * 5}
    _request(node, async_node, action)
    node._action_timer_callback()
    node.snapshot = ObservationSnapshot({"image": 2}, (22, 23))

    def request(data):
        assert set(data) == {"image", "tail"}
        node._action_timer_callback()
        node._action_timer_callback()
        return action

    node.model_inferencer.request_inference = request
    _infer(node, async_node)
    assert node.current_action_idx == 2
    node._action_timer_callback()
    assert _stamp(node.publishers["/base_cmd"].published[-1]) == (22, 23)

    assert _stamp(node.publishers["/base_cmd"].published[-1].poses[0]) == (
        22,
        15_000_023,
    )


def test_async_rejects_unequal_horizons_before_calling_stitcher():
    node = _runtime(async_node, joint=True)
    node._stitcher = Mock(enabled=True)
    _request(node, async_node, {"base": [POSE], "arm": [[1], [2]]})
    node._stitcher.stitch.assert_not_called()
    assert node.current_actions is None
    assert node._action_observation_stamp is None
    assert all(not p.published for p in node.publishers.values())


def test_published_joint_rows_match_real_solver():
    node = _runtime(async_node, joint=True)
    node._stitcher = TrajectoryStitcher(
        TrajectoryStitchConfig(
            max_velocity=3, max_acceleration=30, max_jerk=300
        ),
        200,
        ["arm"],
    )
    if not node._stitcher.enabled:
        pytest.skip("optional osqp/scipy dependencies are not installed")
    action = {
        "base": [POSE] * 21,
        "arm": np.linspace(0, 0.1, 21)[:, None].tolist(),
    }
    original = copy.deepcopy(action)
    _request(node, async_node, action)
    assert node._stitcher.n_solved == 1
    solved = node.current_actions["arm"]
    assert abs(solved[1][0] - original["arm"][1][0]) > 1e-4
    for _ in range(3):
        node._action_timer_callback()
    published = [msg.position for msg in node.publishers["/arm_cmd"].published]
    np.testing.assert_allclose(published, solved[:3], atol=1e-12)
    assert action == original


def test_real_stitcher_pending_handover_keeps_pose_and_request_reference():
    node = _runtime(async_node, joint=True)
    node._stitcher = TrajectoryStitcher(
        TrajectoryStitchConfig(
            max_velocity=1, max_acceleration=10, max_jerk=100
        ),
        200,
        ["arm"],
        node.get_logger(),
    )
    action = {"arm": [[0.1]] * 10, "base": [POSE] * 10}
    _request(node, async_node, action)
    node._action_timer_callback()
    node.snapshot = ObservationSnapshot({"image": 2}, (22, 23))
    _request(node, async_node, action)
    assert node._pending_actions is not None
    assert node._action_observation_stamp == (11, 12)
    assert node._pending_actions[3] == (22, 23)
    node.snapshot = ObservationSnapshot({"image": 3}, (33, 34))
    for _ in range(3):
        node._action_timer_callback()
    assert node.current_actions["base"] == action["base"]
    assert _stamp(node.publishers["/base_cmd"].published[-1]) == (22, 23)


@pytest.mark.parametrize(
    "frequency,offsets",
    [
        (200.0, [5_000_000, 15_000_000]),
        (25.0, [40_000_000, 120_000_000]),
        (30.0, [33_333_333, 100_000_000]),
    ],
)
def test_path_sample_times_preserve_row_indices_and_nanosecond_precision(
    frequency, offsets
):
    node = _FakeNode()
    config = _deploy_config(_channels(joint=False))
    config.control_config.control_frequency = frequency
    executor = ActionExecutor(node, config)
    action = {"base": [POSE] * 3}
    reference = (1_800_000_000, 999_999_999)
    for index in (0, 2):
        executor.send_action(action, index, reference)
    paths = node.publishers_by_topic["/base_cmd"].published
    for path, offset in zip(paths, offsets, strict=True):
        assert _stamp(path) == reference
        assert len(path.poses) == 1
        assert _stamp(path.poses[0]) == (reference[0] + 1, offset - 1)
