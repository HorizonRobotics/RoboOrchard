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

import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)


@dataclass
class ArmState:
    name: str
    current_positions: list[float] | None = None
    current_velocities: list[float] | None = None
    last_state_time_s: float | None = None


@dataclass
class ReplaySample:
    path_s: float
    t_demo_s: float
    left: list[float]
    right: list[float]


class RecordedReplayMasterNode(Node):
    """Replay a measured safe follower path for gravity ID data collection."""

    def __init__(self) -> None:
        super().__init__("recorded_replay_master")

        self.declare_parameter("reference_csv", "")
        self.declare_parameter("tracking_reference_csv", "")
        self.declare_parameter("left_command_topic", "/left_algo_cmd")
        self.declare_parameter("right_command_topic", "/right_algo_cmd")
        self.declare_parameter("left_state_topic", "/puppet/joint_left")
        self.declare_parameter("right_state_topic", "/puppet/joint_right")
        self.declare_parameter("publish_left", True)
        self.declare_parameter("publish_right", True)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_delay_s", 1.0)
        self.declare_parameter("start_trigger_file", "")
        self.declare_parameter("ready_file", "")
        self.declare_parameter("min_command_subscribers", 0)
        self.declare_parameter("command_subscriber_wait_timeout_s", 2.0)
        self.declare_parameter("reset_before_start", False)
        self.declare_parameter(
            "reset_pose", "0.0,0.10,-0.70,0.0,0.0,0.0,0.0"
        )
        self.declare_parameter(
            "reset_param_nodes",
            "/robot/left/robot_left_controller,/robot/right/robot_right_controller",
        )
        self.declare_parameter(
            "reset_services",
            "/robot/left/reset_ctrl,/robot/right/reset_ctrl",
        )
        self.declare_parameter("reset_service_timeout_s", 25.0)
        self.declare_parameter("reset_settle_s", 0.5)
        self.declare_parameter("replay_speed_path_s_per_s", 0.25)
        self.declare_parameter("use_demo_timing", False)
        self.declare_parameter("demo_speed_scale", 1.0)
        self.declare_parameter("reverse", False)
        self.declare_parameter("bidirectional", False)
        self.declare_parameter("approach_start_s", 0.0)
        self.declare_parameter("approach_start_max_error_rad", 1.0)
        self.declare_parameter("hold_start_s", 2.0)
        self.declare_parameter("hold_turnaround_s", 2.0)
        self.declare_parameter("hold_end_s", 2.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("kd", 0.8)
        self.declare_parameter("torque_ref", 0.0)
        self.declare_parameter("use_message_velocity", False)
        self.declare_parameter("state_timeout_s", 0.25)
        self.declare_parameter("tracking_error_stop_rad", 0.25)
        self.declare_parameter("tracking_error_warn_rad", 0.12)
        self.declare_parameter("reference_feedback_gain", 0.0)
        self.declare_parameter("reference_feedback_max_offset_rad", 0.0)
        self.declare_parameter("output_csv", "")
        self.declare_parameter("log_decimation", 1)

        reference_csv = str(self.get_parameter("reference_csv").value or "")
        if not reference_csv:
            raise ValueError("reference_csv is required")
        self.samples = self._load_reference_csv(Path(reference_csv))
        tracking_reference_csv = str(
            self.get_parameter("tracking_reference_csv").value or ""
        )
        self.tracking_samples = None
        if tracking_reference_csv:
            self.tracking_samples = self._load_reference_csv(
                Path(tracking_reference_csv)
            )
        if self.get_parameter("reverse").value:
            self.samples = self._reverse_samples(self.samples)
            if self.tracking_samples is not None:
                self.tracking_samples = self._reverse_samples(
                    self.tracking_samples
                )
        self.path_length_s = self.samples[-1].path_s
        if self.path_length_s <= 0.0:
            raise ValueError("reference_csv path length must be positive")
        if self.tracking_samples is not None:
            tracking_path_length_s = self.tracking_samples[-1].path_s
            if abs(tracking_path_length_s - self.path_length_s) > 1e-3:
                raise ValueError(
                    "tracking_reference_csv path length must match "
                    "reference_csv path length "
                    f"({tracking_path_length_s:.6f} vs "
                    f"{self.path_length_s:.6f})"
                )
        demo_pairs = sorted(
            (sample.t_demo_s, sample.path_s) for sample in self.samples
        )
        self._demo_times = [pair[0] for pair in demo_pairs]
        self._demo_paths = [pair[1] for pair in demo_pairs]
        self.demo_duration_s = max(
            1e-6, self._demo_times[-1] - self._demo_times[0]
        )

        self.publish_left = bool(self.get_parameter("publish_left").value)
        self.publish_right = bool(self.get_parameter("publish_right").value)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.start_delay_s = max(
            0.0, float(self.get_parameter("start_delay_s").value)
        )
        self.start_trigger_file = str(
            self.get_parameter("start_trigger_file").value or ""
        )
        self.ready_file = str(self.get_parameter("ready_file").value or "")
        self.min_command_subscribers = max(
            0, int(self.get_parameter("min_command_subscribers").value)
        )
        self.command_subscriber_wait_timeout_s = max(
            0.0,
            float(
                self.get_parameter(
                    "command_subscriber_wait_timeout_s"
                ).value
            ),
        )
        self.reset_before_start = bool(
            self.get_parameter("reset_before_start").value
        )
        self.reset_pose = self._parse_pose(
            str(self.get_parameter("reset_pose").value or "")
        )
        self.reset_param_nodes = self._parse_csv_list(
            str(self.get_parameter("reset_param_nodes").value or "")
        )
        self.reset_services = self._parse_csv_list(
            str(self.get_parameter("reset_services").value or "")
        )
        self.reset_service_timeout_s = max(
            0.0, float(self.get_parameter("reset_service_timeout_s").value)
        )
        self.reset_settle_s = max(
            0.0, float(self.get_parameter("reset_settle_s").value)
        )
        if self.reset_before_start and not self.reset_services:
            raise ValueError(
                "reset_services is required when reset_before_start=true"
            )
        self.replay_speed = max(
            1e-6,
            float(
                self.get_parameter("replay_speed_path_s_per_s").value
            ),
        )
        self.use_demo_timing = bool(
            self.get_parameter("use_demo_timing").value
        )
        self.demo_speed_scale = max(
            1e-6, float(self.get_parameter("demo_speed_scale").value)
        )
        self.approach_start_s = max(
            0.0, float(self.get_parameter("approach_start_s").value)
        )
        self.approach_start_max_error_rad = max(
            0.0,
            float(self.get_parameter("approach_start_max_error_rad").value),
        )
        self.hold_start_s = max(
            0.0, float(self.get_parameter("hold_start_s").value)
        )
        self.hold_turnaround_s = max(
            0.0, float(self.get_parameter("hold_turnaround_s").value)
        )
        self.hold_end_s = max(
            0.0, float(self.get_parameter("hold_end_s").value)
        )
        self.loop = bool(self.get_parameter("loop").value)
        self.bidirectional = bool(
            self.get_parameter("bidirectional").value
        )
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.torque_ref = float(self.get_parameter("torque_ref").value)
        self.use_message_velocity = bool(
            self.get_parameter("use_message_velocity").value
        )
        self.state_timeout_s = max(
            0.0, float(self.get_parameter("state_timeout_s").value)
        )
        self.tracking_error_stop_rad = max(
            0.0, float(self.get_parameter("tracking_error_stop_rad").value)
        )
        self.tracking_error_warn_rad = max(
            0.0, float(self.get_parameter("tracking_error_warn_rad").value)
        )
        self.reference_feedback_gain = max(
            0.0, float(self.get_parameter("reference_feedback_gain").value)
        )
        self.reference_feedback_max_offset_rad = max(
            0.0,
            float(
                self.get_parameter(
                    "reference_feedback_max_offset_rad"
                ).value
            ),
        )
        self.output_csv = str(self.get_parameter("output_csv").value or "")
        self.log_decimation = max(
            1, int(self.get_parameter("log_decimation").value)
        )

        self.left = ArmState("left")
        self.right = ArmState("right")
        self._started = False
        self._done = False
        self._done_future: Future = Future()
        self._ready_signaled = False
        self._state_wait_logged = False
        self._subscriber_wait_logged = False
        self._tracking_warned = False
        self._subscriber_wait_started_at = None
        self._start_time = self.get_clock().now()
        self._replay_start_time = None
        self._prev_left_cmd: list[float] | None = None
        self._prev_right_cmd: list[float] | None = None
        self._prev_cmd_time_s: float | None = None
        self._approach_left_start: list[float] | None = None
        self._approach_right_start: list[float] | None = None
        self._reset_state = "idle"
        self._reset_stage_start_s: float | None = None
        self._reset_pending: list[tuple[str, Future]] = []
        self._reset_param_clients = {}
        self._reset_service_clients = {}
        self._last_log_cmd_left: list[float] | None = None
        self._last_log_cmd_right: list[float] | None = None
        self._last_log_vel_left: list[float] | None = None
        self._last_log_vel_right: list[float] | None = None
        self._log_count = 0
        self._log_file = None
        self._log_writer = None

        left_command_topic = self.get_parameter("left_command_topic").value
        right_command_topic = self.get_parameter("right_command_topic").value
        left_state_topic = self.get_parameter("left_state_topic").value
        right_state_topic = self.get_parameter("right_state_topic").value

        self.left_pub = self.create_publisher(JointState, left_command_topic, 1)
        self.right_pub = self.create_publisher(JointState, right_command_topic, 1)
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
        for node_name in self.reset_param_nodes:
            self._reset_param_clients[node_name] = self.create_client(
                SetParameters, f"{node_name}/set_parameters"
            )
        for service_name in self.reset_services:
            self._reset_service_clients[service_name] = self.create_client(
                Trigger, service_name
            )

        if self.output_csv:
            self._open_log(Path(self.output_csv))

        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)
        move_duration = self._move_duration_s()
        replay_duration = move_duration
        if self.bidirectional:
            replay_duration = 2.0 * move_duration + self.hold_turnaround_s
        total_duration = (
            self.approach_start_s
            + self.hold_start_s
            + replay_duration
            + self.hold_end_s
        )
        if self.reset_before_start:
            total_duration += (
                self.reset_service_timeout_s + self.reset_settle_s
            )
        self.get_logger().info(
            "Recorded replay loaded "
            f"{len(self.samples)} samples from {reference_csv}; "
            f"path_length={self.path_length_s:.3f}, "
            f"tracking_reference={self.tracking_samples is not None}, "
            f"speed={self.replay_speed:.3f}, "
            f"use_demo_timing={self.use_demo_timing}, "
            f"demo_speed_scale={self.demo_speed_scale:.3f}, "
            f"motion_duration={replay_duration:.1f}s, "
            f"total_duration={total_duration:.1f}s, "
            f"reset_before_start={self.reset_before_start}, "
            f"approach_start_s={self.approach_start_s:.1f}, "
            f"bidirectional={self.bidirectional}, "
            f"reference_feedback_gain={self.reference_feedback_gain:.2f}, "
            f"left={self.publish_left} to {left_command_topic}, "
            f"right={self.publish_right} to {right_command_topic}"
        )

    @staticmethod
    def _parse_csv_list(value: str) -> list[str]:
        return [item.strip() for item in value.split(",") if item.strip()]

    @staticmethod
    def _parse_pose(value: str) -> list[float]:
        cleaned = value.replace("[", " ").replace("]", " ")
        cleaned = cleaned.replace(";", ",").replace(" ", ",")
        pose = [float(item) for item in cleaned.split(",") if item.strip()]
        if len(pose) != len(JOINT_NAMES):
            raise ValueError(
                "reset_pose must have 7 values, "
                f"got {len(pose)} from {value!r}"
            )
        return pose

    @staticmethod
    def _load_reference_csv(path: Path) -> list[ReplaySample]:
        with path.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            required = ["path_s", "t_demo_s"]
            required += [f"left_{name}" for name in JOINT_NAMES]
            required += [f"right_{name}" for name in JOINT_NAMES]
            missing = [name for name in required if name not in reader.fieldnames]
            if missing:
                raise ValueError(
                    f"reference_csv is missing columns: {', '.join(missing)}"
                )
            samples = []
            for row in reader:
                samples.append(
                    ReplaySample(
                        path_s=float(row["path_s"]),
                        t_demo_s=float(row["t_demo_s"]),
                        left=[
                            float(row[f"left_{name}"]) for name in JOINT_NAMES
                        ],
                        right=[
                            float(row[f"right_{name}"]) for name in JOINT_NAMES
                        ],
                    )
                )
        if len(samples) < 2:
            raise ValueError("reference_csv must contain at least two rows")
        samples.sort(key=lambda sample: sample.path_s)
        return samples

    @staticmethod
    def _reverse_samples(samples: list[ReplaySample]) -> list[ReplaySample]:
        total = samples[-1].path_s
        reversed_samples = [
            ReplaySample(
                path_s=total - sample.path_s,
                t_demo_s=sample.t_demo_s,
                left=sample.left,
                right=sample.right,
            )
            for sample in reversed(samples)
        ]
        reversed_samples.sort(key=lambda sample: sample.path_s)
        return reversed_samples

    def _state_callback(self, arm: ArmState, msg: JointState) -> None:
        if len(msg.position) < len(JOINT_NAMES):
            return
        arm.current_positions = [
            float(value) for value in msg.position[: len(JOINT_NAMES)]
        ]
        if len(msg.velocity) >= len(JOINT_NAMES):
            arm.current_velocities = [
                float(value) for value in msg.velocity[: len(JOINT_NAMES)]
            ]
        else:
            arm.current_velocities = [0.0] * len(JOINT_NAMES)
        arm.last_state_time_s = self._now_s()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _setup_ready(self, elapsed: float) -> bool:
        if elapsed < self.start_delay_s:
            return False
        left_ready = (
            (not self.publish_left) or self.left.current_positions is not None
        )
        right_ready = (
            (not self.publish_right) or self.right.current_positions is not None
        )
        if left_ready and right_ready:
            return True
        if not self._state_wait_logged:
            self._state_wait_logged = True
            self.get_logger().warning(
                "Waiting for follower joint state before recorded replay start."
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
            self.get_logger().info("Recorded replay ready file written.")
        except Exception as e:
            self.get_logger().error(f"Failed to write ready file: {e}")

    def _ready_to_start(self, elapsed: float) -> bool:
        if not self._setup_ready(elapsed):
            return False
        if not self._reset_before_ready():
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
        count_text = ", ".join(
            f"{name}={count}" for name, count in counts
        ) or "none"
        if not self._subscriber_wait_logged:
            self._subscriber_wait_logged = True
            self.get_logger().info(
                "Waiting for command subscribers before recorded replay "
                f"start: need {self.min_command_subscribers}, have "
                f"{count_text}."
            )
        if wait_elapsed >= self.command_subscriber_wait_timeout_s:
            self.get_logger().warning(
                "Timed out waiting for command subscribers before recorded "
                f"replay start; have {count_text}, starting anyway."
            )
            return True
        return False

    def _reset_stage_elapsed(self) -> float:
        if self._reset_stage_start_s is None:
            return 0.0
        return self._now_s() - self._reset_stage_start_s

    def _set_reset_state(self, state: str) -> None:
        self._reset_state = state
        self._reset_stage_start_s = self._now_s()

    def _reset_timed_out(self, action: str) -> bool:
        if self.reset_service_timeout_s <= 0.0:
            return False
        elapsed = self._reset_stage_elapsed()
        if elapsed <= self.reset_service_timeout_s:
            return False
        self._stop_with_error(
            f"Reset before replay timed out while {action} "
            f"after {elapsed:.1f}s."
        )
        return True

    def _reset_clients_ready(self, clients: dict, action: str) -> bool:
        missing = []
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=0.0):
                missing.append(name)
        if not missing:
            return True
        if self._reset_timed_out(action):
            return False
        return False

    def _reset_pose_request(self) -> SetParameters.Request:
        value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in self.reset_pose],
        )
        parameter = Parameter(name="reset_joint_position", value=value)
        request = SetParameters.Request()
        request.parameters = [parameter]
        return request

    def _reset_pending_done(self, action: str) -> bool:
        if all(future.done() for _, future in self._reset_pending):
            return True
        if self._reset_timed_out(action):
            return False
        return False

    def _check_reset_param_results(self) -> bool:
        for node_name, future in self._reset_pending:
            if future.exception() is not None:
                self._stop_with_error(
                    "Failed to set reset_joint_position on "
                    f"{node_name}: {future.exception()}"
                )
                return False
            response = future.result()
            for result in response.results:
                if not result.successful:
                    self._stop_with_error(
                        "Failed to set reset_joint_position on "
                        f"{node_name}: {result.reason}"
                    )
                    return False
        return True

    def _check_reset_service_results(self) -> bool:
        for service_name, future in self._reset_pending:
            if future.exception() is not None:
                self._stop_with_error(
                    f"Reset service {service_name} failed: "
                    f"{future.exception()}"
                )
                return False
            response = future.result()
            if not response.success:
                self._stop_with_error(
                    f"Reset service {service_name} returned failure: "
                    f"{response.message}"
                )
                return False
        return True

    def _reset_before_ready(self) -> bool:
        if not self.reset_before_start:
            return True
        if self._reset_state == "done":
            return True
        if self._reset_state == "idle":
            pose_text = ", ".join(f"{v:.3f}" for v in self.reset_pose)
            self.get_logger().info(
                "Resetting follower arms before recorded replay; "
                f"reset_pose=[{pose_text}]."
            )
            if self._reset_param_clients:
                self._set_reset_state("wait_param_services")
            else:
                self._set_reset_state("wait_reset_services")

        if self._reset_state == "wait_param_services":
            if not self._reset_clients_ready(
                self._reset_param_clients, "waiting for parameter services"
            ):
                return False
            self._reset_pending = [
                (node_name, client.call_async(self._reset_pose_request()))
                for node_name, client in self._reset_param_clients.items()
            ]
            self._set_reset_state("set_reset_params")
            return False

        if self._reset_state == "set_reset_params":
            if not self._reset_pending_done("setting reset_joint_position"):
                return False
            if not self._check_reset_param_results():
                return False
            self._set_reset_state("wait_reset_services")
            return False

        if self._reset_state == "wait_reset_services":
            if not self._reset_clients_ready(
                self._reset_service_clients, "waiting for reset services"
            ):
                return False
            self._reset_pending = [
                (service_name, client.call_async(Trigger.Request()))
                for service_name, client in self._reset_service_clients.items()
            ]
            self._set_reset_state("call_reset_services")
            return False

        if self._reset_state == "call_reset_services":
            if not self._reset_pending_done("calling reset services"):
                return False
            if not self._check_reset_service_results():
                return False
            self.get_logger().info(
                "Follower reset services completed; "
                f"settling for {self.reset_settle_s:.2f}s."
            )
            self._set_reset_state("settle")
            return False

        if self._reset_state == "settle":
            if self._reset_stage_elapsed() < self.reset_settle_s:
                return False
            self._set_reset_state("done")
            self.get_logger().info("Follower reset complete.")
            return True

        self._stop_with_error(f"Unknown reset state {self._reset_state!r}.")
        return False

    def _sample_from(
        self, samples: list[ReplaySample], path_s: float
    ) -> ReplaySample:
        path_s = max(0.0, min(self.path_length_s, path_s))
        lo = 0
        hi = len(samples) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if samples[mid].path_s <= path_s:
                lo = mid
            else:
                hi = mid
        a = samples[lo]
        b = samples[min(lo + 1, len(samples) - 1)]
        span = b.path_s - a.path_s
        if span <= 1e-12:
            alpha = 0.0
        else:
            alpha = (path_s - a.path_s) / span
        return ReplaySample(
            path_s=path_s,
            t_demo_s=(1.0 - alpha) * a.t_demo_s + alpha * b.t_demo_s,
            left=[
                (1.0 - alpha) * av + alpha * bv
                for av, bv in zip(a.left, b.left, strict=True)
            ],
            right=[
                (1.0 - alpha) * av + alpha * bv
                for av, bv in zip(a.right, b.right, strict=True)
            ],
        )

    def _sample_at(self, path_s: float) -> ReplaySample:
        return self._sample_from(self.samples, path_s)

    def _tracking_sample_at(self, path_s: float) -> ReplaySample:
        if self.tracking_samples is None:
            return self._sample_at(path_s)
        return self._sample_from(self.tracking_samples, path_s)

    @staticmethod
    def _interpolate_positions(
        start: list[float], target: list[float], alpha: float
    ) -> list[float]:
        alpha = max(0.0, min(1.0, alpha))
        return [
            (1.0 - alpha) * a + alpha * b
            for a, b in zip(start, target, strict=True)
        ]

    def _capture_approach_start(self) -> bool:
        target = self.samples[0]
        errors = []
        if self.publish_left:
            if self.left.current_positions is None:
                self._stop_with_error(
                    "Cannot approach start: left state is unavailable."
                )
                return False
            self._approach_left_start = list(self.left.current_positions)
            errors.extend(
                abs(a - b)
                for a, b in zip(
                    self._approach_left_start[:6], target.left[:6], strict=True
                )
            )
        else:
            self._approach_left_start = list(target.left)

        if self.publish_right:
            if self.right.current_positions is None:
                self._stop_with_error(
                    "Cannot approach start: right state is unavailable."
                )
                return False
            self._approach_right_start = list(self.right.current_positions)
            errors.extend(
                abs(a - b)
                for a, b in zip(
                    self._approach_right_start[:6], target.right[:6], strict=True
                )
            )
        else:
            self._approach_right_start = list(target.right)

        max_error = max(errors) if errors else 0.0
        if (
            self.approach_start_max_error_rad > 0.0
            and max_error > self.approach_start_max_error_rad
        ):
            self._stop_with_error(
                "Cannot approach start: current pose is too far from the "
                f"recorded start pose ({max_error:.4f} rad > "
                f"{self.approach_start_max_error_rad:.4f} rad)."
            )
            return False

        self.get_logger().info(
            "Approaching recorded start pose from current follower state; "
            f"max_start_error={max_error:.4f} rad, "
            f"duration={self.approach_start_s:.2f}s."
        )
        return True

    def _approach_sample(self, alpha: float) -> ReplaySample:
        target = self.samples[0]
        left_start = self._approach_left_start or target.left
        right_start = self._approach_right_start or target.right
        return ReplaySample(
            path_s=0.0,
            t_demo_s=target.t_demo_s,
            left=self._interpolate_positions(left_start, target.left, alpha),
            right=self._interpolate_positions(right_start, target.right, alpha),
        )

    def _command_velocity(
        self,
        current: list[float],
        previous: list[float] | None,
        dt: float | None,
    ) -> list[float]:
        if not self.use_message_velocity or previous is None or dt is None:
            return [0.0] * len(JOINT_NAMES)
        if dt <= 1e-4:
            return [0.0] * len(JOINT_NAMES)
        return [
            max(-45.0, min(45.0, (q - q_prev) / dt))
            for q, q_prev in zip(current, previous, strict=True)
        ]

    def _feedback_command(
        self, reference: list[float], arm: ArmState
    ) -> list[float]:
        if (
            self.reference_feedback_gain <= 0.0
            or self.reference_feedback_max_offset_rad <= 0.0
            or arm.current_positions is None
        ):
            return list(reference)
        command = list(reference)
        for i in range(6):
            error = reference[i] - arm.current_positions[i]
            offset = self.reference_feedback_gain * error
            offset = max(
                -self.reference_feedback_max_offset_rad,
                min(self.reference_feedback_max_offset_rad, offset),
            )
            command[i] = reference[i] + offset
        return command

    def _publish(
        self,
        publisher,
        positions: list[float],
        velocities: list[float],
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = [0.0] * len(JOINT_NAMES)
        publisher.publish(msg)

    def _state_stale(self, arm: ArmState) -> bool:
        if arm.last_state_time_s is None:
            return True
        return (self._now_s() - arm.last_state_time_s) > self.state_timeout_s

    def _max_tracking_error(
        self, positions: list[float], arm: ArmState
    ) -> float | None:
        if arm.current_positions is None:
            return None
        return max(
            abs(cmd - meas)
            for cmd, meas in zip(
                positions[:6], arm.current_positions[:6], strict=True
            )
        )

    def _stop_with_error(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._finish()

    def _validate_runtime_state(
        self,
        left_positions: list[float],
        right_positions: list[float],
    ) -> bool:
        if self.state_timeout_s > 0.0:
            if self.publish_left and self._state_stale(self.left):
                self._stop_with_error("Stopping replay: left state is stale.")
                return False
            if self.publish_right and self._state_stale(self.right):
                self._stop_with_error("Stopping replay: right state is stale.")
                return False

        errors = []
        if self.publish_left:
            left_error = self._max_tracking_error(left_positions, self.left)
            if left_error is not None:
                errors.append(("left", left_error))
        if self.publish_right:
            right_error = self._max_tracking_error(right_positions, self.right)
            if right_error is not None:
                errors.append(("right", right_error))
        for arm_name, error in errors:
            if (
                self.tracking_error_stop_rad > 0.0
                and error > self.tracking_error_stop_rad
            ):
                self._stop_with_error(
                    "Stopping replay: "
                    f"{arm_name} tracking error {error:.4f} rad exceeds "
                    f"{self.tracking_error_stop_rad:.4f} rad."
                )
                return False
            if (
                not self._tracking_warned
                and self.tracking_error_warn_rad > 0.0
                and error > self.tracking_error_warn_rad
            ):
                self._tracking_warned = True
                self.get_logger().warning(
                    f"{arm_name} tracking error reached {error:.4f} rad."
                )
        return True

    def _open_log(self, path: Path) -> None:
        if path.parent:
            path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = path.open("w", newline="", encoding="utf-8")
        self._log_writer = csv.writer(self._log_file)
        header = ["t_unix", "t_replay_s", "path_s", "phase"]
        for side in ("left", "right"):
            for name in JOINT_NAMES:
                header.append(f"{side}_{name}_cmd")
            for name in JOINT_NAMES:
                header.append(f"{side}_{name}_meas")
            for name in JOINT_NAMES:
                header.append(f"{side}_{name}_v_cmd")
            for name in JOINT_NAMES:
                header.append(f"{side}_{name}_v_meas")
            for name in JOINT_NAMES[:6]:
                header.append(f"{side}_{name}_tau_label")
        self._log_writer.writerow(header)

    @staticmethod
    def _arm_values(values: list[float] | None) -> list[float]:
        if values is None:
            return [math.nan] * len(JOINT_NAMES)
        return values[: len(JOINT_NAMES)]

    def _tau_labels(
        self,
        cmd: list[float],
        meas: list[float] | None,
        v_cmd: list[float],
        v_meas: list[float] | None,
    ) -> list[float]:
        if meas is None:
            return [math.nan] * 6
        if v_meas is None:
            v_meas = [0.0] * len(JOINT_NAMES)
        labels = []
        for i in range(6):
            labels.append(
                self.kp * (cmd[i] - meas[i])
                + self.kd * (v_cmd[i] - v_meas[i])
                + self.torque_ref
            )
        return labels

    def _log_row(self, t_replay_s: float, path_s: float, phase: str) -> None:
        if self._log_writer is None:
            return
        self._log_count += 1
        if self._log_count % self.log_decimation != 0:
            return
        left_cmd = self._last_log_cmd_left or [math.nan] * len(JOINT_NAMES)
        right_cmd = self._last_log_cmd_right or [math.nan] * len(JOINT_NAMES)
        left_vel = self._last_log_vel_left or [0.0] * len(JOINT_NAMES)
        right_vel = self._last_log_vel_right or [0.0] * len(JOINT_NAMES)
        left_meas = self._arm_values(self.left.current_positions)
        right_meas = self._arm_values(self.right.current_positions)
        left_v_meas = self._arm_values(self.left.current_velocities)
        right_v_meas = self._arm_values(self.right.current_velocities)

        row = [f"{self._now_s():.9f}", f"{t_replay_s:.6f}", f"{path_s:.9f}", phase]
        row += [f"{x:.9f}" for x in left_cmd]
        row += [f"{x:.9f}" for x in left_meas]
        row += [f"{x:.9f}" for x in left_vel]
        row += [f"{x:.9f}" for x in left_v_meas]
        row += [
            f"{x:.9f}"
            for x in self._tau_labels(
                left_cmd,
                self.left.current_positions,
                left_vel,
                self.left.current_velocities,
            )
        ]
        row += [f"{x:.9f}" for x in right_cmd]
        row += [f"{x:.9f}" for x in right_meas]
        row += [f"{x:.9f}" for x in right_vel]
        row += [f"{x:.9f}" for x in right_v_meas]
        row += [
            f"{x:.9f}"
            for x in self._tau_labels(
                right_cmd,
                self.right.current_positions,
                right_vel,
                self.right.current_velocities,
            )
        ]
        self._log_writer.writerow(row)

    def _finish(self) -> None:
        self._done = True
        if self._log_file is not None:
            self._log_file.flush()
            self._log_file.close()
            self._log_file = None
        if not self._done_future.done():
            self._done_future.set_result(None)

    def _move_duration_s(self) -> float:
        if self.use_demo_timing:
            return self.demo_duration_s / self.demo_speed_scale
        return self.path_length_s / self.replay_speed

    def _path_at_demo_elapsed(self, elapsed_s: float) -> float:
        demo_t = self._demo_times[0] + max(
            0.0, min(self.demo_duration_s, elapsed_s * self.demo_speed_scale)
        )
        lo = 0
        hi = len(self._demo_times) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self._demo_times[mid] <= demo_t:
                lo = mid
            else:
                hi = mid
        ta = self._demo_times[lo]
        tb = self._demo_times[min(lo + 1, len(self._demo_times) - 1)]
        pa = self._demo_paths[lo]
        pb = self._demo_paths[min(lo + 1, len(self._demo_paths) - 1)]
        if tb <= ta:
            return pa
        alpha = (demo_t - ta) / (tb - ta)
        return (1.0 - alpha) * pa + alpha * pb

    def _replay_position(self, t_replay_s: float) -> tuple[float, str, bool]:
        if t_replay_s < self.approach_start_s:
            return 0.0, "approach_start", False
        after_approach_s = t_replay_s - self.approach_start_s
        if after_approach_s < self.hold_start_s:
            return 0.0, "hold_start", False
        move_t = after_approach_s - self.hold_start_s
        move_duration = self._move_duration_s()

        if self.use_demo_timing:
            path_s = self._path_at_demo_elapsed(move_t)
        else:
            path_s = move_t * self.replay_speed
        if move_t <= move_duration:
            return path_s, "move_forward", False

        if self.bidirectional:
            turnaround_elapsed = move_t - move_duration
            if turnaround_elapsed < self.hold_turnaround_s:
                return self.path_length_s, "hold_turnaround", False

            reverse_elapsed = turnaround_elapsed - self.hold_turnaround_s
            if self.use_demo_timing:
                reverse_path_s = self._path_at_demo_elapsed(
                    move_duration - reverse_elapsed
                )
            else:
                reverse_path_s = self.path_length_s - (
                    reverse_elapsed * self.replay_speed
                )
            if reverse_elapsed <= move_duration:
                return reverse_path_s, "move_reverse", False

            end_elapsed = reverse_elapsed - move_duration
            end_path_s = 0.0
        else:
            end_elapsed = move_t - move_duration
            end_path_s = self.path_length_s

        if end_elapsed < self.hold_end_s:
            return end_path_s, "hold_end", False
        if self.loop:
            if self._replay_start_time is not None:
                self._replay_start_time = self.get_clock().now()
                self._prev_left_cmd = None
                self._prev_right_cmd = None
                self._prev_cmd_time_s = None
            return 0.0, "loop_reset", False
        return end_path_s, "done", True

    def _timer_callback(self) -> None:
        if self._done:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if not self._started:
            if not self._ready_to_start(elapsed):
                return
            self._started = True
            if self.approach_start_s > 0.0:
                if not self._capture_approach_start():
                    return
            self._replay_start_time = self.get_clock().now()
            self.get_logger().info("Starting recorded joint replay.")

        assert self._replay_start_time is not None
        t_replay_s = (
            self.get_clock().now() - self._replay_start_time
        ).nanoseconds * 1e-9
        path_s, phase, done = self._replay_position(t_replay_s)
        if phase == "approach_start" and self.approach_start_s > 0.0:
            reference = self._approach_sample(
                t_replay_s / self.approach_start_s
            )
            tracking_reference = reference
        else:
            reference = self._sample_at(path_s)
            tracking_reference = self._tracking_sample_at(path_s)

        left_cmd = self._feedback_command(reference.left, self.left)
        right_cmd = self._feedback_command(reference.right, self.right)

        now_s = self._now_s()
        dt = None
        if self._prev_cmd_time_s is not None:
            dt = now_s - self._prev_cmd_time_s
        left_vel = self._command_velocity(left_cmd, self._prev_left_cmd, dt)
        right_vel = self._command_velocity(right_cmd, self._prev_right_cmd, dt)

        if not self._validate_runtime_state(
            tracking_reference.left, tracking_reference.right
        ):
            self._finish()
            return

        if self.publish_left:
            self._publish(self.left_pub, left_cmd, left_vel)
        if self.publish_right:
            self._publish(self.right_pub, right_cmd, right_vel)

        self._last_log_cmd_left = left_cmd
        self._last_log_cmd_right = right_cmd
        self._last_log_vel_left = left_vel
        self._last_log_vel_right = right_vel
        self._log_row(t_replay_s, path_s, phase)

        self._prev_left_cmd = left_cmd
        self._prev_right_cmd = right_cmd
        self._prev_cmd_time_s = now_s

        if done:
            self.get_logger().info("Recorded joint replay complete.")
            self._finish()


def main(args=None):
    rclpy.init(args=args)
    node = RecordedReplayMasterNode()
    try:
        rclpy.spin_until_future_complete(node, node._done_future)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
