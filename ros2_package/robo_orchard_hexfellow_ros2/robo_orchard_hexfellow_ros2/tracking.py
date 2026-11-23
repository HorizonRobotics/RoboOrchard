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
from collections import deque
from dataclasses import dataclass


def wrap(angle: float) -> float:
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2:
    """Planar pose: x/y in meters, yaw in radians."""

    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError("Pose must be finite")

    def compose(self, relative: "Pose2") -> "Pose2":
        """Transform a body-relative pose into this pose's world frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2(
            self.x + c * relative.x - s * relative.y,
            self.y + s * relative.x + c * relative.y,
            wrap(self.yaw + relative.yaw),
        )


@dataclass(frozen=True)
class Settings:
    """Controller gains, SI-unit bounds and receipt-time timeouts (seconds)."""

    history_duration: float = 35.0
    command_timeout: float = 0.5
    odom_timeout: float = 0.5
    status_timeout: float = 3.0
    max_linear_speed: float = 0.3
    max_angular_speed: float = 0.3
    longitudinal_gain: float = 2.0
    lateral_gain: float = 4.0
    heading_gain: float = 4.0
    reset_distance: float = 1.0
    reset_angle: float = 1.0

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if value <= 0:
                raise ValueError(f"{name} must be positive")


def feedback(
    current: Pose2,
    goal: Pose2,
    reference_velocity: tuple[float, float],
    settings: Settings,
) -> tuple[float, float]:
    """Return bounded (m/s, rad/s) feedforward plus body-error feedback.

    Reference velocity is signed forward speed and yaw rate. Lateral
    feedback is coupled to forward speed, so a stationary reference does
    not correct pure lateral error. This is not a global pose regulator.
    """
    dx, dy = goal.x - current.x, goal.y - current.y
    c, s = math.cos(current.yaw), math.sin(current.yaw)
    ex, ey = c * dx + s * dy, -s * dx + c * dy
    yaw_error = wrap(goal.yaw - current.yaw)
    vr, wr = reference_velocity
    v = vr * math.cos(yaw_error) + settings.longitudinal_gain * ex
    omega = (
        wr
        + settings.lateral_gain * vr * ey
        + settings.heading_gain * math.sin(yaw_error)
    )
    # Scale both together: independent clipping changes the curvature and
    # changes the requested trajectory when angular speed is limiting.
    scale = max(
        1.0,
        abs(v) / settings.max_linear_speed,
        abs(omega) / settings.max_angular_speed,
    )
    return v / scale, omega / scale


class OdomHistory:
    """Duration-bounded history with interpolation, never extrapolation."""

    def __init__(self, duration: float) -> None:
        self.duration = duration
        self.samples: deque[tuple[int, Pose2]] = deque()

    def add(self, stamp: int, pose: Pose2) -> None:
        if self.samples and stamp == self.samples[-1][0]:
            self.samples.pop()
        self.samples.append((stamp, pose))
        cutoff = stamp - int(self.duration * 1e9)
        # Retain one sample before the cutoff to bracket the boundary.
        while len(self.samples) > 2 and self.samples[1][0] < cutoff:
            self.samples.popleft()

    def lookup(self, stamp: int) -> Pose2:
        if not self.samples or not (
            max(
                self.samples[0][0],
                self.samples[-1][0] - int(self.duration * 1e9),
            )
            <= stamp
            <= self.samples[-1][0]
        ):
            raise ValueError("Observation stamp outside odometry history")
        for index, (time, pose) in enumerate(self.samples):
            if stamp == time:
                return pose
            if stamp < time:
                before_time, before = self.samples[index - 1]
                fraction = (stamp - before_time) / (time - before_time)
                return Pose2(
                    before.x + fraction * (pose.x - before.x),
                    before.y + fraction * (pose.y - before.y),
                    wrap(before.yaw + fraction * wrap(pose.yaw - before.yaw)),
                )
        raise ValueError("No bracketing odometry")


class Tracker:
    """Feedback state gated by authoritative manager and inference snapshots.

    All `now` arguments are monotonic receipt times in seconds. Observation
    and odometry stamps are integer ROS nanoseconds, not freshness clocks.
    This class does not choose or change a control mode.
    """

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.history = OdomHistory(settings.history_duration)
        self.current: Pose2 | None = None
        self.odom_received = -math.inf
        self.mode = ""
        self.mode_received = -math.inf
        self.enabled = False
        self.status_received = -math.inf
        self.target_received = -math.inf
        self.teleop_received = -math.inf
        self.goal: Pose2 | None = None
        self.sample_stamp: int | None = None
        self.reference_velocity = (0.0, 0.0)
        self.reference: tuple[int, Pose2] | None = None
        self.cancelled_through: int | None = None

    def cancel(self) -> None:
        """Invalidate the target and its latched observation origin."""
        if self.reference is not None:
            stamp = self.reference[0]
            self.cancelled_through = (
                stamp
                if self.cancelled_through is None
                else max(stamp, self.cancelled_through)
            )
        self.goal = None
        self.sample_stamp = None
        self.reference_velocity = (0.0, 0.0)
        self.reference = None

    def mode_update(self, mode: str, now: float) -> bool:
        """Record manager mode; return whether a transition needs a stop."""
        changed = mode != self.mode
        if changed or not self.mode_fresh(now):
            self.cancel()
            self.teleop_received = -math.inf
        self.mode, self.mode_received = mode, now
        if mode != "auto":
            self.cancel()
        return changed

    def status_update(self, enabled: bool, now: float) -> None:
        if (
            not enabled
            or now - self.status_received > self.settings.status_timeout
        ):
            self.cancel()
        self.enabled, self.status_received = enabled, now

    def mode_fresh(self, now: float) -> bool:
        return 0 <= now - self.mode_received <= self.settings.status_timeout

    def auto_ready(self, now: float) -> bool:
        return (
            self.mode == "auto"
            and self.mode_fresh(now)
            and self.enabled
            and 0 <= now - self.status_received <= self.settings.status_timeout
            and self.current is not None
            and 0 <= now - self.odom_received <= self.settings.odom_timeout
        )

    def reset_odometry(self) -> None:
        """Fail closed after invalid odometry or a coordinate discontinuity."""
        self.history.samples.clear()
        self.current = None
        self.cancel()
        # A ROS clock rollback starts a new timestamp epoch.
        self.cancelled_through = None

    def odom_update(self, stamp: int, pose: Pose2, now: float) -> bool:
        """Cache odometry; report whether tracking continuity was lost."""
        reset = False
        if self.history.samples:
            previous_stamp, previous = self.history.samples[-1]
            reset = (
                stamp < previous_stamp
                or math.hypot(pose.x - previous.x, pose.y - previous.y)
                > self.settings.reset_distance
                or abs(wrap(pose.yaw - previous.yaw))
                > self.settings.reset_angle
            )
            if reset:
                self.reset_odometry()
            elif now - self.odom_received > self.settings.odom_timeout:
                # The ROS clock is continuous; keep cancelled origins invalid.
                self.cancel()
                self.history.samples.clear()
                reset = True
        self.history.add(stamp, pose)
        self.current, self.odom_received = pose, now
        return reset

    def target_update(
        self,
        stamp: int,
        relative: Pose2,
        now: float,
        *,
        sample_stamp: int,
    ) -> None:
        """Latch origin once, then compose each frame independently.

        Raises ValueError and cancels tracking if state/history is unavailable.
        ``stamp`` is the observation reference in ROS nanoseconds;
        ``sample_stamp`` is the model sample time in ROS nanoseconds.
        A new observation starts with zero feedforward. Within one observation,
        differences use sample times, independent of skipped targets or receipt
        jitter. Duplicate/out-of-order samples are ignored without refreshing
        the command timeout or replacing the latest target.
        """
        try:
            if not self.auto_ready(now):
                raise ValueError(
                    "Autonomous tracking state is not fresh/ready"
                )
            if (
                self.cancelled_through is not None
                and stamp <= self.cancelled_through
            ):
                raise ValueError("Target belongs to a cancelled observation")
            if self.reference is not None and stamp < self.reference[0]:
                raise ValueError("Target observation moved backwards")
            if sample_stamp <= stamp:
                raise ValueError("Sample time must follow observation time")
            if self.reference is None or self.reference[0] != stamp:
                self.reference = (stamp, self.history.lookup(stamp))
                self.goal = None
                self.sample_stamp = None
            if (
                self.sample_stamp is not None
                and sample_stamp <= self.sample_stamp
            ):
                return
            goal = self.reference[1].compose(relative)
            self.reference_velocity = (0.0, 0.0)
            if self.goal is not None:
                dt = (sample_stamp - self.sample_stamp) / 1_000_000_000
                dx, dy = goal.x - self.goal.x, goal.y - self.goal.y
                self.reference_velocity = (
                    (
                        math.cos(self.goal.yaw) * dx
                        + math.sin(self.goal.yaw) * dy
                    )
                    / dt,
                    wrap(goal.yaw - self.goal.yaw) / dt,
                )
            self.goal = goal
            self.sample_stamp = sample_stamp
            self.target_received = now
        except ValueError:
            self.cancel()
            raise

    def teleop_allowed(self, now: float) -> bool:
        """Require fresh TAKEOVER, independent of inference and odometry."""
        return self.mode == "takeover" and self.mode_fresh(now)

    def command(self, now: float) -> tuple[float, float] | None:
        """Return velocity/stop, or None to leave fresh teleop alone.

        Consume the latest reference velocity once. Without another target,
        subsequent ticks retain pose feedback but do not extrapolate motion.
        """
        if self.teleop_allowed(now):
            self.cancel()
            if (
                0
                <= now - self.teleop_received
                <= self.settings.command_timeout
            ):
                return None
            return (0.0, 0.0)
        if (
            not self.auto_ready(now)
            or not 0
            <= now - self.target_received
            <= self.settings.command_timeout
        ):
            self.cancel()
        if self.goal is None or self.current is None:
            return (0.0, 0.0)
        command = feedback(
            self.current, self.goal, self.reference_velocity, self.settings
        )
        if not all(math.isfinite(value) for value in command):
            self.cancel()
            return (0.0, 0.0)
        self.reference_velocity = (0.0, 0.0)
        return command
