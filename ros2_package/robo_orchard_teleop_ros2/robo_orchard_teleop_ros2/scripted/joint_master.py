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

"""Publish deterministic joint trajectories for Piper evaluation."""

import json
import math
import os
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from robo_orchard_teleop_ros2.robot_eval import load_scenario
from robo_orchard_teleop_ros2.robot_eval.results import ResultWriter

JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)

DEFAULT_CENTER = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_AMPLITUDES = (0.12, 0.10, 0.10, 0.12, 0.08, 0.10, 0.0)
DEFAULT_FREQUENCIES = (0.07, 0.11, 0.09, 0.13, 0.10, 0.15, 0.0)
DEFAULT_PHASES = (0.0, -1.5708, 3.1416, 0.7854, 2.3562, 3.9270, 0.0)


@dataclass
class ArmState:
    """Track one arm's measured state and trajectory center."""

    name: str
    current_positions: list[float] | None = None
    trajectory_center: list[float] | None = None


class ScriptedJointMasterNode(Node):
    """Publish validated scripted trajectories to Piper follower arms."""

    def __init__(self) -> None:
        super().__init__("scripted_joint_master")

        self.declare_parameter("scenario", "")
        self.declare_parameter("scenario_directory", "")
        self.declare_parameter("result_path", "")
        self.scenario_name = str(self.get_parameter("scenario").value or "")
        scenario_directory = str(
            self.get_parameter("scenario_directory").value or ""
        )
        scenario_params = {}
        if self.scenario_name:
            scenario = load_scenario(
                self.scenario_name,
                directory=scenario_directory or None,
            )
            scenario_params = scenario.ros_parameters()
            self.get_logger().info(
                f"Loaded robot-eval scenario {scenario.name!r} from "
                f"{scenario.path}."
            )

        self.declare_parameter("left_command_topic", "/left_algo_cmd")
        self.declare_parameter("right_command_topic", "/right_algo_cmd")
        self.declare_parameter("left_state_topic", "/puppet/joint_left")
        self.declare_parameter("right_state_topic", "/puppet/joint_right")
        self.declare_parameter("publish_left", True)
        self.declare_parameter("publish_right", True)
        self.declare_parameter(
            "use_current_state",
            scenario_params.get("use_current_state", True),
        )
        self.declare_parameter("wait_for_state_timeout_s", 2.0)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_delay_s", 1.0)
        self.declare_parameter("start_trigger_file", "")
        self.declare_parameter("ready_file", "")
        self.declare_parameter("use_start_position", False)
        self.declare_parameter("start_position_left", list(DEFAULT_CENTER))
        self.declare_parameter("start_position_right", list(DEFAULT_CENTER))
        self.declare_parameter("min_command_subscribers", 0)
        self.declare_parameter("command_subscriber_wait_timeout_s", 2.0)
        # Motion mode: sine sweep, lockstepped waypoints, or planned circle.
        self.declare_parameter("mode", scenario_params.get("mode", "sine"))
        # Circle plans are speed-validated; speed_scale can only slow them.
        self.declare_parameter(
            "circle_trajectory_left",
            scenario_params.get(
                "circle_trajectory_left",
                "/data/holobrain/circle_test/circle_traj_left.json",
            ),
        )
        self.declare_parameter(
            "circle_trajectory_right",
            scenario_params.get(
                "circle_trajectory_right",
                "/data/holobrain/circle_test/circle_traj_right.json",
            ),
        )
        # Seven values per waypoint; both arms must have equal counts.
        self.declare_parameter(
            "waypoints_left", scenario_params.get("waypoints_left", [0.0])
        )
        self.declare_parameter(
            "waypoints_right", scenario_params.get("waypoints_right", [0.0])
        )
        # Peak joint velocity (rad/s) used to size segment duration.
        self.declare_parameter(
            "peak_velocity", scenario_params.get("peak_velocity", 2.0)
        )
        # Global slowdown for staged bring-up (0.3 -> 0.6 -> 1.0).
        self.declare_parameter(
            "speed_scale", scenario_params.get("speed_scale", 0.3)
        )
        # Peak joint acceleration cap (rad/s^2).
        self.declare_parameter(
            "max_accel", scenario_params.get("max_accel", 8.0)
        )
        # Dwell at each waypoint (s).
        self.declare_parameter(
            "settle_s", scenario_params.get("settle_s", 0.4)
        )
        # One-based waypoint indices that skip dwell for thermal safety.
        self.declare_parameter(
            "pass_through_indices",
            scenario_params.get("pass_through_indices", [0]),
        )
        # Number of forward+reverse laps through the waypoint list.
        self.declare_parameter("laps", scenario_params.get("laps", 1))
        self.declare_parameter("ramp_s", 0.0)
        self.declare_parameter(
            "duration_s", scenario_params.get("duration_s", 10.0)
        )
        self.declare_parameter(
            "amplitude_scale",
            scenario_params.get("amplitude_scale", 1.0),
        )
        self.declare_parameter(
            "frequency_scale",
            scenario_params.get("frequency_scale", 1.0),
        )
        self.declare_parameter("mirror_right", False)
        self.declare_parameter("center_left", list(DEFAULT_CENTER))
        self.declare_parameter("center_right", list(DEFAULT_CENTER))
        self.declare_parameter("amplitudes", list(DEFAULT_AMPLITUDES))
        self.declare_parameter("frequencies", list(DEFAULT_FREQUENCIES))
        self.declare_parameter("phases", list(DEFAULT_PHASES))

        self.publish_left = self.get_parameter("publish_left").value
        self.publish_right = self.get_parameter("publish_right").value
        self.use_current_state = self.get_parameter("use_current_state").value
        self.wait_for_state_timeout_s = float(
            self.get_parameter("wait_for_state_timeout_s").value
        )
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.start_delay_s = max(
            0.0, float(self.get_parameter("start_delay_s").value)
        )
        self.start_trigger_file = str(
            self.get_parameter("start_trigger_file").value or ""
        )
        self.ready_file = str(self.get_parameter("ready_file").value or "")
        self.use_start_position = bool(
            self.get_parameter("use_start_position").value
        )
        self.min_command_subscribers = max(
            0, int(self.get_parameter("min_command_subscribers").value)
        )
        self.command_subscriber_wait_timeout_s = max(
            0.0,
            float(
                self.get_parameter("command_subscriber_wait_timeout_s").value
            ),
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("sine", "waypoints", "circle"):
            raise ValueError("mode must be 'sine', 'waypoints' or 'circle'")
        self.peak_velocity = max(
            0.05, float(self.get_parameter("peak_velocity").value)
        )
        self.speed_scale = max(
            0.01, float(self.get_parameter("speed_scale").value)
        )
        self.max_accel = max(0.1, float(self.get_parameter("max_accel").value))
        self.settle_s = max(0.0, float(self.get_parameter("settle_s").value))
        self.pass_through_indices = {
            int(v)
            for v in (self.get_parameter("pass_through_indices").value or [])
            if int(v) > 0
        }
        self.laps = max(1, int(self.get_parameter("laps").value))
        self.waypoints_left = self._parse_waypoints("waypoints_left")
        self.waypoints_right = self._parse_waypoints("waypoints_right")
        if self.mode == "waypoints":
            if not self.waypoints_left or not self.waypoints_right:
                raise ValueError(
                    "waypoints mode needs waypoints_left and waypoints_right"
                )
            if len(self.waypoints_left) != len(self.waypoints_right):
                raise ValueError(
                    "waypoints_left and waypoints_right must have the same "
                    "waypoint count (segments are lockstepped across arms "
                    "so only demonstrated pose combinations are visited)."
                )
        # Built lazily at start (needs current joint state for the ramp-in).
        self._wp_plan: (
            list[
                tuple[
                    float,
                    float,
                    list[float] | None,
                    list[float],
                    list[float] | None,
                    list[float],
                ]
            ]
            | None
        ) = None
        self._wp_total_s = 0.0
        # Load circle files at startup; build the timeline after state arrives.
        self._circle_left: dict | None = None
        self._circle_right: dict | None = None
        if self.mode == "circle":
            if self.publish_left:
                self._circle_left = self._load_circle_trajectory(
                    "circle_trajectory_left", "left"
                )
            if self.publish_right:
                self._circle_right = self._load_circle_trajectory(
                    "circle_trajectory_right", "right"
                )
        self._circle_ramp: list[
            tuple[
                float,
                float,
                list[float],
                list[float],
                list[float],
                list[float],
            ]
        ] = []
        self._circle_play_t0 = 0.0
        self._circle_speed = 1.0
        self._circle_total_s = 0.0
        self._circle_idx = [0, 0]
        self.amplitude_scale = float(
            self.get_parameter("amplitude_scale").value
        )
        self.frequency_scale = max(
            0.0, float(self.get_parameter("frequency_scale").value)
        )
        self.mirror_right = self.get_parameter("mirror_right").value

        self.center_left = self._joint_array_parameter("center_left")
        self.center_right = self._joint_array_parameter("center_right")
        self.start_position_left = self._joint_array_parameter(
            "start_position_left"
        )
        self.start_position_right = self._joint_array_parameter(
            "start_position_right"
        )
        self.amplitudes = self._joint_array_parameter("amplitudes")
        self.frequencies = self._joint_array_parameter("frequencies")
        self.phases = self._joint_array_parameter("phases")

        self.left = ArmState("left")
        self.right = ArmState("right")
        result_path = str(self.get_parameter("result_path").value or "")
        self._result_writer = (
            ResultWriter(
                result_path,
                scenario=self.scenario_name or self.mode,
                mode=self.mode,
            )
            if result_path
            else None
        )
        if self._result_writer is not None:
            self.get_logger().info(
                f"Robot-eval results -> {self._result_writer.path}"
            )
        self._started = False
        self._done = False
        self._done_future: Future = Future()
        self._state_wait_logged = False
        self._subscriber_wait_logged = False
        self._subscriber_wait_started_at = None
        self._ready_signaled = False
        self._start_time = self.get_clock().now()

        left_command_topic = self.get_parameter("left_command_topic").value
        right_command_topic = self.get_parameter("right_command_topic").value
        left_state_topic = self.get_parameter("left_state_topic").value
        right_state_topic = self.get_parameter("right_state_topic").value

        self.left_pub = self.create_publisher(
            JointState, left_command_topic, 1
        )
        self.right_pub = self.create_publisher(
            JointState, right_command_topic, 1
        )

        self.create_subscription(
            JointState,
            left_state_topic,
            lambda msg: self._state_callback(self.left, msg),
            1,
        )
        self.create_subscription(
            JointState,
            right_state_topic,
            lambda msg: self._state_callback(self.right, msg),
            1,
        )

        self.timer = self.create_timer(
            1.0 / self.rate_hz, self._timer_callback
        )
        self.get_logger().info(
            "Scripted joint master publishing "
            f"left={self.publish_left} to {left_command_topic}, "
            f"right={self.publish_right} to {right_command_topic}, "
            f"rate={self.rate_hz:.1f} Hz, duration={self.duration_s:.1f}s"
        )

    def _joint_array_parameter(self, name: str) -> list[float]:
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != len(JOINT_NAMES):
            raise ValueError(
                f"Parameter {name} must have {len(JOINT_NAMES)} values; "
                f"got {len(values)}"
            )
        return values

    def _state_callback(self, arm: ArmState, msg: JointState) -> None:
        if len(msg.position) < len(JOINT_NAMES):
            return
        arm.current_positions = [
            float(value) for value in msg.position[: len(JOINT_NAMES)]
        ]

    def _setup_ready(self, elapsed: float) -> bool:
        if elapsed < self.start_delay_s:
            return False
        if not self.use_current_state:
            return True

        left_ready = (
            not self.publish_left
        ) or self.left.current_positions is not None
        right_ready = (
            not self.publish_right
        ) or self.right.current_positions is not None
        if left_ready and right_ready:
            return True

        if (
            not self._state_wait_logged
            and elapsed >= self.start_delay_s + self.wait_for_state_timeout_s
        ):
            self._state_wait_logged = True
            self.get_logger().warning(
                "Waiting for follower joint state before starting scripted "
                "motion; no command will be published until the current "
                "position is known."
            )
        return False

    def _signal_ready(self) -> None:
        if self._ready_signaled:
            return
        self._ready_signaled = True
        if not self.ready_file:
            return
        try:
            ready_dir = os.path.dirname(self.ready_file)
            if ready_dir:
                os.makedirs(ready_dir, exist_ok=True)
            with open(self.ready_file, "a", encoding="utf-8"):
                os.utime(self.ready_file, None)
            self.get_logger().info(
                "Scripted joint master setup complete; ready file written."
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to write scripted motion ready file: {e}"
            )

    def _ready_to_start(self, elapsed: float) -> bool:
        if not self._setup_ready(elapsed):
            return False
        self._signal_ready()
        if self.start_trigger_file and not os.path.exists(
            self.start_trigger_file
        ):
            return False
        if not self._command_subscribers_ready():
            return False
        return True

    def _command_subscriber_counts(self) -> list[tuple[str, int]]:
        counts = []
        if self.publish_left:
            counts.append(("left", self.left_pub.get_subscription_count()))
        if self.publish_right:
            counts.append(("right", self.right_pub.get_subscription_count()))
        return counts

    def _command_subscribers_ready(self) -> bool:
        if self.min_command_subscribers <= 0:
            return True

        counts = self._command_subscriber_counts()
        if all(count >= self.min_command_subscribers for _, count in counts):
            return True

        now = self.get_clock().now()
        if self._subscriber_wait_started_at is None:
            self._subscriber_wait_started_at = now

        wait_elapsed = (
            now - self._subscriber_wait_started_at
        ).nanoseconds * 1e-9
        count_text = (
            ", ".join(f"{name}={count}" for name, count in counts) or "none"
        )
        if not self._subscriber_wait_logged:
            self._subscriber_wait_logged = True
            self.get_logger().info(
                "Waiting for command subscribers before scripted motion "
                f"start: need {self.min_command_subscribers}, have "
                f"{count_text}."
            )

        if wait_elapsed >= self.command_subscriber_wait_timeout_s:
            self.get_logger().warning(
                "Timed out waiting for command subscribers before scripted "
                f"motion start after {wait_elapsed:.2f}s; have "
                f"{count_text}, starting anyway."
            )
            return True

        return False

    def _parse_waypoints(self, name: str) -> list[list[float]]:
        """Parse flattened seven-value waypoint rows."""
        values = [float(v) for v in (self.get_parameter(name).value or [])]
        if len(values) <= 1:
            return []
        if len(values) % 7 != 0:
            raise ValueError(
                f"{name} must be a flat list with 7 values per waypoint "
                f"(6 joints + gripper); got {len(values)}."
            )
        return [values[i : i + 7] for i in range(0, len(values), 7)]

    @staticmethod
    def _minjerk(s: float) -> float:
        return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)

    def _segment_duration(
        self,
        a_left: list[float],
        b_left: list[float],
        a_right: list[float],
        b_right: list[float],
        peak_velocity: float,
        max_accel: float,
    ) -> float:
        dq = 0.0
        for a, b in ((a_left, b_left), (a_right, b_right)):
            for j in range(6):
                dq = max(dq, abs(b[j] - a[j]))
        if dq < 1e-6:
            return 0.0
        t_vel = 1.875 * dq / peak_velocity
        t_acc = math.sqrt(5.7735 * dq / max_accel)
        return max(0.3, t_vel, t_acc)

    def _build_waypoint_plan(self) -> None:
        """Precompute lockstepped min-jerk waypoint segments."""
        wl, wr = self.waypoints_left, self.waypoints_right
        v_peak = self.peak_velocity * self.speed_scale
        a_peak = self.max_accel * self.speed_scale
        cur_l = list(self.left.current_positions or wl[0])
        cur_r = list(self.right.current_positions or wr[0])
        if len(cur_l) < 7:
            cur_l = cur_l + [0.0] * (7 - len(cur_l))
        if len(cur_r) < 7:
            cur_r = cur_r + [0.0] * (7 - len(cur_r))

        plan = []
        t = 0.0

        def add_move(la, lb, ra, rb, v, a):
            nonlocal t
            duration = self._segment_duration(la, lb, ra, rb, v, a)
            if duration > 0.0:
                plan.append((t, duration, la, lb, ra, rb))
                t += duration

        def add_dwell(l_pose, r_pose, wp_index_1based):
            nonlocal t
            if wp_index_1based in self.pass_through_indices:
                return
            if self.settle_s > 0.0:
                plan.append((t, self.settle_s, l_pose, l_pose, r_pose, r_pose))
                t += self.settle_s

        # Slow ramp-in regardless of speed_scale.
        add_move(cur_l, wl[0], cur_r, wr[0], 0.25, 1.0)
        add_dwell(wl[0], wr[0], 1)
        n = len(wl)
        for _ in range(self.laps):
            for k in range(1, n):
                add_move(wl[k - 1], wl[k], wr[k - 1], wr[k], v_peak, a_peak)
                add_dwell(wl[k], wr[k], k + 1)
            for k in range(n - 2, -1, -1):
                add_move(wl[k + 1], wl[k], wr[k + 1], wr[k], v_peak, a_peak)
                add_dwell(wl[k], wr[k], k + 1)
        self._wp_plan = plan
        self._wp_total_s = t
        self.get_logger().info(
            f"Waypoint plan: {n} waypoints x {self.laps} lap(s), "
            f"{len(plan)} segments, {t:.1f} s total at speed_scale "
            f"{self.speed_scale} (peak {v_peak:.2f} rad/s), "
            f"pass-through: {sorted(self.pass_through_indices) or 'none'}."
        )

    def _waypoint_positions(
        self, elapsed: float
    ) -> tuple[list[float], list[float]]:
        plan = self._wp_plan or []
        if not plan:
            return list(self.waypoints_left[0]), list(self.waypoints_right[0])
        last = plan[-1]
        if elapsed >= last[0] + last[1]:
            return list(last[3]), list(last[5])
        seg = plan[0]
        for candidate in plan:
            if candidate[0] <= elapsed:
                seg = candidate
            else:
                break
        t0, duration, la, lb, ra, rb = seg
        s = self._minjerk(min(1.0, max(0.0, (elapsed - t0) / duration)))
        left = [la[j] + (lb[j] - la[j]) * s for j in range(7)]
        right = [ra[j] + (rb[j] - ra[j]) * s for j in range(7)]
        return left, right

    def _load_circle_trajectory(self, param: str, side: str) -> dict:
        """Load and validate a planned circle trajectory."""
        path = str(self.get_parameter(param).value or "")
        if not path:
            raise ValueError(
                f"circle mode needs {param} (a robot-eval circle plan "
                "JSON) for every published arm."
            )
        try:
            with open(path, encoding="utf-8") as fh:
                traj = json.load(fh)
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"failed to load circle trajectory {path!r}: {exc}. "
                "Run `robot-eval plan circle --side left|right` first."
            ) from exc
        ts = traj.get("t") or []
        qs = traj.get("q") or []
        qds = traj.get("qd") or []
        if (
            len(ts) < 2
            or len(qs) != len(ts)
            or len(qds) != len(ts)
            or any(len(row) < 6 for row in qs)
            or any(b <= a for a, b in zip(ts, ts[1:], strict=False))
        ):
            raise ValueError(
                f"circle trajectory {path!r} is malformed: needs "
                "equal-length t/q/qd arrays with strictly increasing t "
                "and 6 joint values per sample."
            )
        if traj.get("side") != side:
            raise ValueError(
                f"circle trajectory {path!r} was planned for side "
                f"{traj.get('side')!r}, expected {side!r}."
            )
        return traj

    def _build_circle_plan(self) -> None:
        """Build the circle ramp-in and playback timeline."""
        speed = min(1.0, self.speed_scale)
        if self.speed_scale > 1.0:
            self.get_logger().warning(
                f"circle mode clamps speed_scale {self.speed_scale:g} "
                "to 1.0 (plans are speed-validated at plan time; only "
                "slowing down is safe)."
            )
        self._circle_speed = speed

        def start_pose(traj, current):
            if traj is None:
                return list(current)
            pose = [float(v) for v in traj["q"][0][:6]]
            # The plan has no gripper column; hold the current opening.
            return pose + [current[6] if len(current) > 6 else 0.0]

        cur_l = list(
            self.left.current_positions
            or start_pose(self._circle_left, [0.0] * 7)
        )
        cur_r = list(
            self.right.current_positions
            or start_pose(self._circle_right, [0.0] * 7)
        )
        cur_l += [0.0] * (7 - len(cur_l))
        cur_r += [0.0] * (7 - len(cur_r))
        start_l = start_pose(self._circle_left, cur_l)
        start_r = start_pose(self._circle_right, cur_r)

        plan = []
        t = 0.0
        # Ramp in slowly regardless of speed_scale.
        ramp_duration = self._segment_duration(
            cur_l, start_l, cur_r, start_r, 0.25, 1.0
        )
        if ramp_duration > 0.0:
            plan.append((t, ramp_duration, cur_l, start_l, cur_r, start_r))
            t += ramp_duration
        if self.settle_s > 0.0:
            plan.append((t, self.settle_s, start_l, start_l, start_r, start_r))
            t += self.settle_s
        self._circle_ramp = plan
        self._circle_play_t0 = t
        self._circle_idx = [0, 0]

        play_s = 0.0
        for traj in (self._circle_left, self._circle_right):
            if traj is not None:
                play_s = max(play_s, float(traj["t"][-1]) / speed)
        self._circle_total_s = t + play_s
        self.get_logger().info(
            f"Circle plan: ramp-in {ramp_duration:.1f} s + settle "
            f"{self.settle_s:.1f} s + playback {play_s:.1f} s at "
            f"speed_scale {speed:g} = {self._circle_total_s:.1f} s "
            f"total (left={'yes' if self._circle_left else 'no'}, "
            f"right={'yes' if self._circle_right else 'no'})."
        )

    def _circle_sample(
        self,
        traj: dict | None,
        arm_index: int,
        play_t: float,
        hold: list[float],
    ) -> tuple[list[float], list[float]]:
        """Interpolate one arm's circle trajectory."""
        if traj is None:
            return list(hold), [0.0] * len(JOINT_NAMES)
        ts, qs, qds = traj["t"], traj["q"], traj["qd"]
        gripper = hold[6] if len(hold) > 6 else 0.0
        if play_t >= ts[-1]:
            return (
                [float(v) for v in qs[-1][:6]] + [gripper],
                [0.0] * len(JOINT_NAMES),
            )
        # Monotonic cursor: elapsed only moves forward.
        idx = self._circle_idx[arm_index]
        while idx + 1 < len(ts) and ts[idx + 1] <= play_t:
            idx += 1
        self._circle_idx[arm_index] = idx
        a = (play_t - ts[idx]) / (ts[idx + 1] - ts[idx])
        position = [
            (1.0 - a) * qs[idx][j] + a * qs[idx + 1][j] for j in range(6)
        ] + [gripper]
        # Chain rule: playback runs at speed_scale of plan time.
        velocity = [
            ((1.0 - a) * qds[idx][j] + a * qds[idx + 1][j])
            * self._circle_speed
            for j in range(6)
        ] + [0.0]
        return position, velocity

    def _circle_positions(
        self, elapsed: float
    ) -> tuple[list[float], list[float], list[float], list[float]]:
        for t0, duration, la, lb, ra, rb in self._circle_ramp:
            if elapsed < t0 + duration:
                s = self._minjerk(
                    min(1.0, max(0.0, (elapsed - t0) / duration))
                )
                zero = [0.0] * len(JOINT_NAMES)
                left = [la[j] + (lb[j] - la[j]) * s for j in range(7)]
                right = [ra[j] + (rb[j] - ra[j]) * s for j in range(7)]
                return left, right, zero, zero
        play_t = (elapsed - self._circle_play_t0) * self._circle_speed
        hold_l = (
            self._circle_ramp[-1][3]
            if self._circle_ramp
            else (list(self.left.current_positions or [0.0] * 7))
        )
        hold_r = (
            self._circle_ramp[-1][5]
            if self._circle_ramp
            else (list(self.right.current_positions or [0.0] * 7))
        )
        left, left_vel = self._circle_sample(
            self._circle_left, 0, play_t, hold_l
        )
        right, right_vel = self._circle_sample(
            self._circle_right, 1, play_t, hold_r
        )
        return left, right, left_vel, right_vel

    def _trajectory_offset_at_zero(self, right: bool) -> list[float]:
        offsets = []
        for idx, (amplitude, phase) in enumerate(
            zip(self.amplitudes, self.phases, strict=True)
        ):
            sign = -1.0 if right and self.mirror_right and idx < 6 else 1.0
            offsets.append(
                sign * self.amplitude_scale * amplitude * math.sin(phase)
            )
        return offsets

    def _initialize_trajectory_center(
        self, arm: ArmState, nominal_center: list[float], right: bool
    ) -> list[float]:
        if arm.trajectory_center is not None:
            return arm.trajectory_center

        if self.use_start_position:
            start_position = (
                self.start_position_right
                if right
                else self.start_position_left
            )
            offsets = self._trajectory_offset_at_zero(right=right)
            arm.trajectory_center = [
                position - offset
                for position, offset in zip(
                    start_position, offsets, strict=True
                )
            ]
            self.get_logger().info(
                f"{arm.name} scripted trajectory starts at configured "
                f"position: {start_position}"
            )
        elif self.use_current_state and arm.current_positions is not None:
            offsets = self._trajectory_offset_at_zero(right=right)
            arm.trajectory_center = [
                current_position - offset
                for current_position, offset in zip(
                    arm.current_positions, offsets, strict=True
                )
            ]
            self.get_logger().info(
                f"{arm.name} scripted trajectory starts at current state: "
                f"{arm.current_positions}"
            )
        else:
            arm.trajectory_center = nominal_center.copy()
            self.get_logger().info(
                f"{arm.name} scripted trajectory uses configured center: "
                f"{arm.trajectory_center}"
            )
        return arm.trajectory_center

    def _trajectory(
        self, center: list[float], t: float, right: bool
    ) -> list[float]:
        positions = []
        for idx, (base, amplitude, frequency, phase) in enumerate(
            zip(
                center,
                self.amplitudes,
                self.frequencies,
                self.phases,
                strict=True,
            )
        ):
            sign = -1.0 if right and self.mirror_right and idx < 6 else 1.0
            offset = (
                sign
                * self.amplitude_scale
                * amplitude
                * math.sin(
                    2.0 * math.pi * frequency * self.frequency_scale * t
                    + phase
                )
            )
            positions.append(base + offset)
        return positions

    def _publish(
        self,
        publisher,
        positions: list[float],
        velocities: list[float] | None = None,
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        # Circle velocity is used only by the message feedforward source.
        msg.velocity = (
            list(velocities)
            if velocities is not None
            else [0.0] * len(JOINT_NAMES)
        )
        msg.effort = [0.0] * len(JOINT_NAMES)
        publisher.publish(msg)

    def _record_result(
        self,
        elapsed: float,
        left_positions: list[float] | None,
        right_positions: list[float] | None,
        *,
        phase: str = "trajectory",
        trajectory_time: float | None = None,
    ) -> None:
        if self._result_writer is None:
            return
        self._result_writer.write(
            elapsed,
            left_command=left_positions if self.publish_left else None,
            left_measured=self.left.current_positions,
            right_command=right_positions if self.publish_right else None,
            right_measured=self.right.current_positions,
            phase=phase,
            trajectory_time=trajectory_time,
        )

    def close_result(self) -> None:
        if self._result_writer is not None:
            self._result_writer.close()
            self._result_writer = None

    def _timer_callback(self) -> None:
        if self._done:
            return

        elapsed = (
            self.get_clock().now() - self._start_time
        ).nanoseconds * 1e-9
        if not self._started:
            if not self._ready_to_start(elapsed):
                return
            self._started = True
            if self.publish_left:
                self._initialize_trajectory_center(
                    self.left, self.center_left, right=False
                )
            if self.publish_right:
                self._initialize_trajectory_center(
                    self.right, self.center_right, right=True
                )
            if self.mode == "waypoints":
                self._build_waypoint_plan()
            elif self.mode == "circle":
                self._build_circle_plan()
            self._start_time = self.get_clock().now()
            elapsed = 0.0
            self.get_logger().info("Starting scripted joint trajectory.")

        if self.mode == "circle":
            done = elapsed > self._circle_total_s or (
                self.duration_s > 0.0 and elapsed > self.duration_s
            )
            if done:
                self._done = True
                self.get_logger().info("Circle trajectory complete.")
                if not self._done_future.done():
                    self._done_future.set_result(None)
                return
            left_positions, right_positions, left_vel, right_vel = (
                self._circle_positions(elapsed)
            )
            if self.publish_left:
                self._publish(self.left_pub, left_positions, left_vel)
            if self.publish_right:
                self._publish(self.right_pub, right_positions, right_vel)
            play_t = max(
                0.0,
                (elapsed - self._circle_play_t0) * self._circle_speed,
            )
            self._record_result(
                elapsed,
                left_positions,
                right_positions,
                phase=(
                    "playback"
                    if elapsed >= self._circle_play_t0
                    else "ramp_in"
                ),
                trajectory_time=play_t,
            )
            return

        if self.mode == "waypoints":
            done = elapsed > self._wp_total_s or (
                self.duration_s > 0.0 and elapsed > self.duration_s
            )
            if done:
                self._done = True
                self.get_logger().info("Waypoint trajectory complete.")
                if not self._done_future.done():
                    self._done_future.set_result(None)
                return
            left_positions, right_positions = self._waypoint_positions(elapsed)
            if self.publish_left:
                self._publish(self.left_pub, left_positions)
            if self.publish_right:
                self._publish(self.right_pub, right_positions)
            self._record_result(
                elapsed,
                left_positions,
                right_positions,
                trajectory_time=elapsed,
            )
            return

        if self.duration_s > 0.0 and elapsed > self.duration_s:
            self._done = True
            self.get_logger().info("Scripted joint trajectory complete.")
            if not self._done_future.done():
                self._done_future.set_result(None)
            return

        left_positions = None
        right_positions = None
        if self.publish_left:
            left_positions = self._trajectory(
                self.left.trajectory_center or self.center_left,
                elapsed,
                right=False,
            )
            self._publish(self.left_pub, left_positions)
        if self.publish_right:
            right_positions = self._trajectory(
                self.right.trajectory_center or self.center_right,
                elapsed,
                right=True,
            )
            self._publish(self.right_pub, right_positions)
        self._record_result(
            elapsed,
            left_positions,
            right_positions,
            trajectory_time=elapsed,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedJointMasterNode()
    try:
        rclpy.spin_until_future_complete(node, node._done_future)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
