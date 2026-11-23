# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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

import math

import pytest

from robo_orchard_hexfellow_ros2.tracking import (
    OdomHistory,
    Pose2,
    Settings,
    Tracker,
    feedback,
    wrap,
)


def ready(settings: Settings | None = None) -> Tracker:
    tracker = Tracker(settings or Settings())
    tracker.mode_update("auto", 40)
    tracker.status_update(True, 40)
    tracker.odom_update(10_000_000_000, Pose2(0, 0, 0), 40)
    return tracker


def test_history_interpolation_wrap_and_no_extrapolation():
    history = OdomHistory(35)
    history.add(0, Pose2(0, 2, math.radians(179)))
    history.add(2_000_000_000, Pose2(2, 4, math.radians(-179)))
    pose = history.lookup(1_000_000_000)
    assert (pose.x, pose.y) == (1, 3)
    assert abs(pose.yaw) == pytest.approx(math.pi)
    assert history.lookup(0).x == 0
    for stamp in (-1, 2_000_000_001):
        with pytest.raises(ValueError):
            history.lookup(stamp)
    history.add(2_000_000_000, Pose2(3, 4, 0))
    assert len(history.samples) == 2
    assert history.lookup(2_000_000_000).x == 3


def test_reference_latched_after_pruning_and_new_chunk_not_a_delta():
    tracker = ready(Settings(history_duration=1))
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(0.1, 0, 0), 40, sample_stamp=stamp + 5_000_000
    )
    for index in range(1, 5):
        tracker.odom_update(
            stamp + index * 1_000_000_000, Pose2(index * 0.1, 0, 0), 40
        )
    with pytest.raises(ValueError):
        tracker.history.lookup(stamp)
    tracker.target_update(
        stamp, Pose2(0.2, 0, 0), 40.1, sample_stamp=stamp + 10_000_000
    )
    assert tracker.goal.x == pytest.approx(0.2)
    tracker.target_update(
        stamp + 4_000_000_000,
        Pose2(0.2, 0, 0),
        40.1,
        sample_stamp=stamp + 4_015_000_000,
    )
    assert tracker.goal.x == pytest.approx(0.6)
    with pytest.raises(ValueError):
        tracker.target_update(
            stamp - 1, Pose2(1, 0, 0), 40.1, sample_stamp=stamp + 19_999_999
        )
    assert tracker.goal is None


def test_body_reference_rotation_and_old_stamp_receipt_freshness():
    tracker = ready()
    tracker.odom_update(10_000_000_000, Pose2(2, 3, math.pi / 2), 40)
    tracker.target_update(
        10_000_000_000, Pose2(1, 0, 0.2), 40, sample_stamp=10_005_000_000
    )
    assert tracker.goal.x == pytest.approx(2)
    assert tracker.goal.y == pytest.approx(4)
    assert tracker.goal.yaw == pytest.approx(math.pi / 2 + 0.2)
    assert tracker.command(40.1)[0] > 0
    assert tracker.command(40.51) == (0, 0)
    assert tracker.goal is None


@pytest.mark.parametrize(
    "cause", ["mode", "disabled", "status", "odom", "target"]
)
def test_cancel_never_resumes_old_target(cause):
    tracker = ready()
    tracker.target_update(
        10_000_000_000, Pose2(0.199, 0, 0), 40, sample_stamp=10_005_000_000
    )
    tracker.target_update(
        10_000_000_000, Pose2(0.2, 0, 0), 40, sample_stamp=10_010_000_000
    )
    assert tracker.reference_velocity[0] > 0
    now = 40.1
    if cause == "mode":
        tracker.mode_update("takeover", now)
        tracker.mode_update("auto", now)
    elif cause == "disabled":
        tracker.status_update(False, now)
        tracker.status_update(True, now)
    else:
        now = 44 if cause == "status" else 40.6
        if cause != "odom":
            tracker.odom_update(10_000_000_000, Pose2(0, 0, 0), now)
        if cause != "target":
            tracker.target_received = now
        tracker.command(now)
        tracker.mode_update("auto", now)
        tracker.status_update(True, now)
        tracker.odom_update(10_000_000_000, Pose2(0, 0, 0), now)
    assert tracker.command(now) == (0, 0)
    assert tracker.goal is None
    assert tracker.reference_velocity == (0, 0)


@pytest.mark.parametrize(
    "stamp,pose",
    [
        (9_000_000_000, Pose2(0, 0, 0)),
        (11_000_000_000, Pose2(2, 0, 0)),
        (11_000_000_000, Pose2(0, 0, 2)),
    ],
)
def test_odometry_reset_cancels_and_clears_old_history(stamp, pose):
    tracker = ready()
    tracker.target_update(
        10_000_000_000, Pose2(0.999, 0, 0), 40, sample_stamp=10_005_000_000
    )
    tracker.target_update(
        10_000_000_000, Pose2(1, 0, 0), 40, sample_stamp=10_010_000_000
    )
    assert tracker.odom_update(stamp, pose, 40.1)
    assert tracker.goal is None
    assert tracker.reference_velocity == (0, 0)
    assert list(tracker.history.samples) == [(stamp, pose)]


def test_odometry_gap_breaks_interpolation_and_fresh_samples_recover():
    tracker = ready()
    start = 10_000_000_000
    tracker.target_update(start, Pose2(0.1, 0, 0), 40, sample_stamp=start + 1)
    # No timer tick during the outage: the first recovered odom must cancel.
    recovered = start + 2_000_000_000
    assert tracker.odom_update(recovered, Pose2(0.6, 0, 0), 42)
    assert tracker.cancelled_through == start
    assert tracker.command(42) == (0, 0)
    gap_stamp = start + 1_000_000_000
    with pytest.raises(ValueError, match="outside odometry history"):
        tracker.target_update(
            gap_stamp, Pose2(0, 0, 0), 42, sample_stamp=gap_stamp + 1
        )
    assert tracker.command(42) == (0, 0)
    # Exactly the configured timeout remains continuous, as in auto_ready.
    assert not tracker.odom_update(
        recovered + 500_000_000, Pose2(0.7, 0, 0), 42.5
    )
    stamp = recovered + 250_000_000
    tracker.target_update(
        stamp, Pose2(0.1, 0, 0), 42.5, sample_stamp=stamp + 1
    )
    assert tracker.goal.x == pytest.approx(0.75)
    assert tracker.command(42.5)[0] > 0


def test_teleop_needs_no_inference_or_odometry_and_timer_does_not_override():
    tracker = Tracker(Settings())
    tracker.mode_update("takeover", 10)
    assert tracker.teleop_allowed(10)
    tracker.teleop_received = 10
    assert tracker.command(10.1) is None
    tracker.status_update(False, 10.1)
    assert tracker.command(10.2) is None
    assert tracker.command(10.6) == (0, 0)
    tracker.teleop_received = 12.9
    assert tracker.command(13.1) == (0, 0)
    assert not tracker.teleop_allowed(13.1)


@pytest.mark.parametrize(
    "settings",
    [
        {"command_timeout": 0},
        {"max_linear_speed": math.inf},
        {"longitudinal_gain": -1},
        {"lateral_gain": 0},
        {"heading_gain": math.nan},
    ],
)
def test_reject_bad_settings(settings):
    with pytest.raises(ValueError):
        Settings(**settings)


def integrate(pose: Pose2, v: float, omega: float, dt: float) -> Pose2:
    return Pose2(
        pose.x + v * math.cos(pose.yaw) * dt,
        pose.y + v * math.sin(pose.yaw) * dt,
        wrap(pose.yaw + omega * dt),
    )


@pytest.mark.parametrize("direction", [1, -1])
def test_tiny_targets_and_lateral_error_have_no_bearing_turn(direction):
    settings = Settings()
    pose = Pose2(0, 0, 0)
    goal = Pose2(direction * 0.0001, 0.001, 0)
    assert feedback(pose, goal, (0, 0), settings) == pytest.approx(
        (direction * 0.0002, 0)
    )
    assert feedback(
        pose, goal, (direction * 0.1, 0), settings
    ) == pytest.approx((direction * 0.1002, direction * 0.0004))


def test_body_error_feedback_and_common_speed_scaling():
    settings = Settings()
    current = Pose2(1, 2, math.pi / 2)
    goal = current.compose(Pose2(0.2, 0.1, 0.1))
    v, w = feedback(current, goal, (0.2, 0.1), settings)
    raw_v = 0.2 * math.cos(0.1) + 2 * 0.2
    raw_w = 0.1 + 4 * 0.2 * 0.1 + 4 * math.sin(0.1)
    scale = max(raw_v / 0.3, raw_w / 0.3)
    assert (v, w) == pytest.approx((raw_v / scale, raw_w / scale))


def test_sample_interval_not_observation_stamp_or_receipt_jitter():
    tracker = ready()
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(0, 0, 0), 40, sample_stamp=stamp + 10_000_000
    )
    assert tracker.reference_velocity == (0, 0)
    tracker.target_update(
        stamp, Pose2(0.001, 0, 0.002), 40.09, sample_stamp=stamp + 20_000_000
    )
    assert tracker.reference_velocity == pytest.approx((0.1, 0.2))
    tracker.target_update(
        stamp, Pose2(0.002, 0, 0.004), 40.091, sample_stamp=stamp + 30_000_000
    )
    assert tracker.reference_velocity == pytest.approx(
        (0.1 * math.cos(0.002), 0.2)
    )
    # No output-side differentiation: each tick consumes the latest sample.
    assert tracker.command(40.092) != tracker.command(40.093)
    assert tracker.reference_velocity == (0, 0)
    # Repeated poses mean zero reference velocity, not duplicate rejection.
    tracker.target_update(
        stamp, Pose2(0.002, 0, 0.004), 40.1, sample_stamp=stamp + 40_000_000
    )
    assert tracker.reference_velocity == (0, 0)


def test_new_chunk_and_cancel_clear_feedforward():
    tracker = ready()
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(0, 0, math.pi - 0.001), 40, sample_stamp=stamp + 5_000_000
    )
    tracker.target_update(
        stamp,
        Pose2(-0.001, 0, -math.pi + 0.001),
        40,
        sample_stamp=stamp + 10_000_000,
    )
    assert tracker.reference_velocity == pytest.approx(
        (0.2 * math.cos(0.001), 0.4)
    )
    tracker.odom_update(stamp + 1, Pose2(0, 0, 0), 40)
    tracker.target_update(
        stamp + 1, Pose2(0.5, 0, 0), 40, sample_stamp=stamp + 15_000_001
    )
    assert tracker.reference_velocity == (0, 0)
    tracker.target_update(
        stamp + 1, Pose2(0.501, 0, 0), 40, sample_stamp=stamp + 20_000_001
    )
    assert tracker.reference_velocity[0] == pytest.approx(0.2)
    tracker.cancel()
    assert tracker.goal is None
    assert tracker.reference_velocity == (0, 0)


def test_missing_target_tick_consumes_feedforward_but_keeps_pose_feedback():
    tracker = ready()
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(-0.0005, 0, 0), 40, sample_stamp=stamp + 5_000_000
    )
    tracker.target_update(
        stamp, Pose2(0, 0, 0), 40.01, sample_stamp=stamp + 10_000_000
    )
    assert tracker.command(40.02) == pytest.approx((0.1, 0))
    assert tracker.command(40.03) == (0, 0)
    assert tracker.goal == Pose2(0, 0, 0)
    # Consuming velocity does not discard the previous target for derivatives.
    tracker.target_update(
        stamp, Pose2(0.0005, 0, 0), 40.04, sample_stamp=stamp + 15_000_000
    )
    assert tracker.command(40.04) == pytest.approx((0.101, 0))
    assert tracker.command(40.05) == pytest.approx((0.001, 0))
    tracker.odom_update(stamp + 1, Pose2(0, 0, 0), 40.5)
    assert tracker.command(40.5) == pytest.approx((0.001, 0))
    assert tracker.command(40.541) == (0, 0)
    assert tracker.reference_velocity == (0, 0)


def test_cancelled_observation_cannot_restart_after_auto_returns():
    tracker = ready()
    tracker.target_update(
        10_000_000_000, Pose2(0.1, 0, 0), 40, sample_stamp=10_005_000_000
    )
    tracker.mode_update("stop", 40.1)
    tracker.mode_update("auto", 40.2)
    with pytest.raises(ValueError, match="cancelled"):
        tracker.target_update(
            10_000_000_000, Pose2(0.2, 0, 0), 40.2, sample_stamp=10_010_000_000
        )
    tracker.odom_update(10_100_000_000, Pose2(0.01, 0, 0), 40.2)
    tracker.target_update(
        10_100_000_000, Pose2(0.1, 0, 0), 40.2, sample_stamp=10_115_000_000
    )
    assert tracker.command(40.2)[0] > 0


@pytest.mark.parametrize(
    "speed,turn", [(0.12, 0), (0.12, 0.08), (-0.12, -0.08)]
)
def test_200hz_targets_100hz_tracker_follows_moving_path(speed, turn):
    tracker = ready()
    stamp = 10_000_000_000
    goal = Pose2(0, 0, 0)
    # Latch the observation before introducing a current-pose disturbance.
    tracker.target_update(stamp, goal, 40, sample_stamp=stamp + 5_000_000)
    # A millimeter lateral disturbance must not trigger a large bearing turn.
    pose = Pose2(0, 0.001, 0)
    dt = 0.005
    errors = []
    speeds = []
    command = (0, 0)
    for index in range(2000):
        now = 40 + index * dt
        tracker.mode_update("auto", now)
        tracker.status_update(True, now)
        tracker.odom_update(stamp + index * 5_000_000, pose, now)
        tracker.target_update(
            stamp, goal, now, sample_stamp=stamp + (index + 2) * 5_000_000
        )
        if index % 2 == 0:
            command = tracker.command(now)
        assert abs(command[0]) <= 0.3 and abs(command[1]) <= 0.3
        pose = integrate(pose, *command, dt)
        goal = integrate(goal, speed, turn, dt)
        if index > 200:
            errors.append(math.hypot(pose.x - goal.x, pose.y - goal.y))
            speeds.append(command[0])
    assert max(errors) < 0.003
    assert all(v * speed > 0 for v in speeds)
    assert abs(wrap(pose.yaw - goal.yaw)) < 0.003


@pytest.mark.parametrize("skip", [1, 2, 5])
def test_skipped_samples_preserve_forward_and_yaw_feedforward(skip):
    tracker = ready()
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(0, 0, 0), 40, sample_stamp=stamp + 5_000_000
    )
    # 0.2 m/s and 0.1 rad/s sampled every 5 ms; callback jitter is irrelevant.
    goal = Pose2(skip * 0.001, 0, skip * 0.0005)
    tracker.target_update(
        stamp, goal, 40.123, sample_stamp=stamp + (skip + 1) * 5_000_000
    )
    assert tracker.reference_velocity == pytest.approx((0.2, 0.1))
    tracker.odom_update(stamp + 200_000_000, goal, 40.123)
    assert tracker.command(40.123) == pytest.approx((0.2, 0.1))


@pytest.mark.parametrize("offset", [5_000_000, 10_000_000])
def test_old_sample_does_not_replace_goal_or_refresh_freshness(offset):
    tracker = ready()
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(0, 0, 0), 40, sample_stamp=stamp + 5_000_000
    )
    tracker.target_update(
        stamp, Pose2(0.001, 0, 0), 40.1, sample_stamp=stamp + 10_000_000
    )
    tracker.target_update(
        stamp, Pose2(1, 0, 0), 40.2, sample_stamp=stamp + offset
    )
    assert tracker.goal == Pose2(0.001, 0, 0)
    assert tracker.sample_stamp == stamp + 10_000_000
    assert tracker.target_received == 40.1
    assert tracker.reference_velocity == pytest.approx((0.2, 0))


@pytest.mark.parametrize("offset", [0, -1])
def test_sample_must_follow_observation(offset):
    tracker = ready()
    stamp = 10_000_000_000
    with pytest.raises(ValueError, match="Sample time"):
        tracker.target_update(
            stamp, Pose2(0, 0, 0), 40, sample_stamp=stamp + offset
        )
    assert tracker.goal is None


def test_new_observation_resets_sample_time_even_when_next_time_is_earlier():
    tracker = ready()
    stamp = 10_000_000_000
    tracker.target_update(
        stamp, Pose2(0, 0, 0), 40, sample_stamp=stamp + 500_000_000
    )
    next_stamp = stamp + 100_000_000
    tracker.odom_update(next_stamp, Pose2(0.02, 0, 0), 40.1)
    tracker.target_update(
        next_stamp,
        Pose2(0.001, 0, 0),
        40.1,
        sample_stamp=next_stamp + 5_000_000,
    )
    assert tracker.goal.x == pytest.approx(0.021)
    assert tracker.reference_velocity == (0, 0)
    tracker.target_update(
        next_stamp,
        Pose2(0.003, 0, 0),
        40.2,
        sample_stamp=next_stamp + 15_000_000,
    )
    assert tracker.reference_velocity == pytest.approx((0.2, 0))
    tracker.cancel()
    assert tracker.sample_stamp is None
