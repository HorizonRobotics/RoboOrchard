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
import json
import os
import shlex
import subprocess
import tempfile
import time
from pathlib import Path

import streamlit as st

from robo_orchard_inference_app.utils import start_process, stop_process
from robo_orchard_teleop_ros2.robot_eval import (
    Scenario,
    ScenarioError,
    list_scenarios,
    load_scenario,
)

_SCRIPTED_MOTION_READY_TIMEOUT_S = 20.0


class ScriptedMotionMixin:
    @staticmethod
    def _format_process_output(output) -> str:
        if output is None:
            return ""
        if isinstance(output, bytes):
            return output.decode(errors="replace").strip()
        return str(output).strip()

    @staticmethod
    def _format_command(command: list[str]) -> str:
        return " ".join(shlex.quote(part) for part in command)

    def _set_scripted_motion_error(self, message: str | None) -> None:
        st.session_state.scripted_motion_error = message

    def _render_scripted_motion_error(self) -> None:
        error = st.session_state.get("scripted_motion_error")
        if not error:
            return

        st.error("Scripted motion failed. Full error details:")
        st.code(error, language="text")
        if st.button(
            "Clear Scripted Motion Error",
            key=f"{self.key_prefix}_clear_scripted_motion_error",
            use_container_width=True,
        ):
            self._set_scripted_motion_error(None)
            st.rerun()

    def _scripted_motion_process(self) -> subprocess.Popen | None:
        process = st.session_state.get("scripted_motion_process")
        if process is None:
            return None

        if process.poll() is None:
            return process

        st.session_state.scripted_motion_process = None
        self._clear_scripted_motion_sync_files()
        return None

    def _clear_scripted_motion_sync_files(self) -> None:
        for state_key, description in (
            ("scripted_motion_trigger_file", "trigger"),
            ("scripted_motion_ready_file", "ready"),
        ):
            sync_file = st.session_state.pop(state_key, None)
            if not sync_file:
                continue
            try:
                os.remove(sync_file)
            except FileNotFoundError:
                pass
            except Exception as e:
                self.logger.warning(
                    f"Failed to remove scripted motion {description} file: {e}"
                )

    def _is_scripted_motion_running(self) -> bool:
        return self._scripted_motion_process() is not None

    @staticmethod
    def _scripted_param_value(
        value: bool | float | int | list[float] | tuple[float, ...] | str,
    ) -> str:
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, float):
            return str(float(value))
        if isinstance(value, int):
            return str(value)
        if isinstance(value, (list, tuple)):
            return (
                "["
                + ", ".join(
                    ScriptedMotionMixin._scripted_param_value(item)
                    for item in value
                )
                + "]"
            )
        return value

    def _scripted_motion_args(
        self,
        duration_s: float,
        amplitude_scale: float,
        frequency_scale: float,
        start_delay_s: float | None = None,
        start_trigger_file: str | None = None,
        ready_file: str | None = None,
        use_current_state: bool | None = None,
        use_start_position: bool | None = None,
        start_position_left: list[float] | None = None,
        start_position_right: list[float] | None = None,
        min_command_subscribers: int | None = None,
        command_subscriber_wait_timeout_s: float | None = None,
        extra_params: dict[str, object] | None = None,
    ) -> list[str]:
        cfg = self.launch_cfg.scripted_motion
        params: dict[
            str, bool | float | int | list[float] | tuple[float, ...] | str
        ] = {
            "left_command_topic": cfg.left_command_topic,
            "right_command_topic": cfg.right_command_topic,
            "left_state_topic": cfg.left_state_topic,
            "right_state_topic": cfg.right_state_topic,
            "publish_left": cfg.publish_left,
            "publish_right": cfg.publish_right,
            "use_current_state": (
                cfg.use_current_state
                if use_current_state is None
                else bool(use_current_state)
            ),
            "mirror_right": cfg.mirror_right,
            "rate_hz": cfg.rate_hz,
            "start_delay_s": (
                cfg.start_delay_s
                if start_delay_s is None
                else float(start_delay_s)
            ),
            "duration_s": duration_s,
            "amplitude_scale": amplitude_scale,
            "frequency_scale": frequency_scale,
        }
        if start_trigger_file:
            params["start_trigger_file"] = start_trigger_file
        if ready_file:
            params["ready_file"] = ready_file
        if use_start_position is not None:
            params["use_start_position"] = bool(use_start_position)
        if start_position_left is not None:
            params["start_position_left"] = [
                float(value) for value in start_position_left
            ]
        if start_position_right is not None:
            params["start_position_right"] = [
                float(value) for value in start_position_right
            ]
        if min_command_subscribers is not None:
            params["min_command_subscribers"] = max(
                0, int(min_command_subscribers)
            )
        if command_subscriber_wait_timeout_s is not None:
            params["command_subscriber_wait_timeout_s"] = max(
                0.0, float(command_subscriber_wait_timeout_s)
            )
        if extra_params:
            params.update(extra_params)

        args = ["--ros-args"]
        for name, value in params.items():
            param_value = self._scripted_param_value(value)
            args.extend(["-p", f"{name}:={param_value}"])
        return args

    def _scripted_motion_base_commands(self) -> list[list[str]]:
        cfg = self.launch_cfg.scripted_motion
        candidates: list[list[str]] = []

        repo_root = Path(__file__).resolve().parents[4]
        script_path = repo_root / (
            "ros2_package/robo_orchard_teleop_ros2/"
            "robo_orchard_teleop_ros2/scripted/joint_master.py"
        )
        install_setup_path = repo_root / "ros2_package/install/setup.bash"
        if script_path.exists():
            if install_setup_path.exists():
                candidates.append(
                    [
                        "bash",
                        "-lc",
                        "source "
                        + shlex.quote(str(install_setup_path))
                        + " && exec python3 "
                        + shlex.quote(str(script_path))
                        + ' "$@"',
                        "scripted_joint_master",
                    ]
                )
            else:
                candidates.append(["python3", str(script_path)])

        if cfg.command:
            if install_setup_path.exists():
                candidates.append(
                    [
                        "bash",
                        "-lc",
                        "source "
                        + shlex.quote(str(install_setup_path))
                        + " && exec "
                        + " ".join(shlex.quote(part) for part in cfg.command)
                        + ' "$@"',
                        "scripted_joint_master",
                    ]
                )
            else:
                candidates.append(list(cfg.command))

        candidates.append(
            [
                "python3",
                "-m",
                "robo_orchard_teleop_ros2.scripted.joint_master",
            ]
        )

        unique_candidates = []
        seen = set()
        for candidate in candidates:
            key = tuple(candidate)
            if key in seen:
                continue
            seen.add(key)
            unique_candidates.append(candidate)
        return unique_candidates

    def _scripted_motion_commands(
        self,
        duration_s: float,
        amplitude_scale: float,
        frequency_scale: float,
        start_delay_s: float | None = None,
        start_trigger_file: str | None = None,
        ready_file: str | None = None,
        use_current_state: bool | None = None,
        use_start_position: bool | None = None,
        start_position_left: list[float] | None = None,
        start_position_right: list[float] | None = None,
        min_command_subscribers: int | None = None,
        command_subscriber_wait_timeout_s: float | None = None,
        extra_params: dict[str, object] | None = None,
    ) -> list[list[str]]:
        args = self._scripted_motion_args(
            duration_s=duration_s,
            amplitude_scale=amplitude_scale,
            frequency_scale=frequency_scale,
            start_delay_s=start_delay_s,
            start_trigger_file=start_trigger_file,
            ready_file=ready_file,
            use_current_state=use_current_state,
            use_start_position=use_start_position,
            start_position_left=start_position_left,
            start_position_right=start_position_right,
            min_command_subscribers=min_command_subscribers,
            command_subscriber_wait_timeout_s=(
                command_subscriber_wait_timeout_s
            ),
            extra_params=extra_params,
        )
        return [base + args for base in self._scripted_motion_base_commands()]

    def _wait_for_scripted_motion_ready(
        self, ready_file: str, process: subprocess.Popen
    ) -> bool:
        timeout_s = _SCRIPTED_MOTION_READY_TIMEOUT_S + max(
            0.0, float(self.launch_cfg.scripted_motion.start_delay_s)
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if os.path.exists(ready_file):
                return True
            if process.poll() is not None:
                message = (
                    "Scripted motion exited before setup completed; "
                    "recording was not started."
                )
                self._set_scripted_motion_error(message)
                self.logger.error(message)
                return False
            time.sleep(0.05)

        message = (
            "Timed out waiting for scripted motion setup to finish "
            f"after {timeout_s:.1f}s; recording was not started."
        )
        self._set_scripted_motion_error(message)
        self.logger.error(message)
        return False

    def _stop_scripted_motion(self) -> bool:
        process = self._scripted_motion_process()
        if process is None:
            st.session_state.scripted_motion_process = None
            self._clear_scripted_motion_sync_files()
            return True

        try:
            stop_process(process, timeout=5.0)
        except Exception as e:
            self.logger.error(f"Failed to stop scripted motion: {e}")
            return False

        st.session_state.scripted_motion_process = None
        self._clear_scripted_motion_sync_files()
        self._set_scripted_motion_error(None)
        self.logger.info("Scripted motion stopped.")
        return True

    def _stop_scripted_motion_and_robot(self) -> None:
        self.ros_helper.set_control_mode("stop")
        self._stop_scripted_motion()
        self.ros_helper.set_control_mode("stop")

    def _scripted_motion_reset_position(self) -> list[float]:
        scripted_reset = self.launch_cfg.scripted_motion.reset_position
        if scripted_reset:
            return [float(value) for value in scripted_reset]

        ros_reset = self.launch_cfg.ros_bridge.reset_position
        if ros_reset:
            return [float(value) for value in ros_reset]

        return [0.0] * 7

    def _stop_scripted_motion_callback(self):
        self._stop_scripted_motion_and_robot()
        scripted_reset = self.launch_cfg.scripted_motion.reset_position
        self.ros_helper.reset_arm(position=scripted_reset or None)

    def _scenario_directory(self) -> str | None:
        configured = self.launch_cfg.scripted_motion.scenario_directory
        return configured or None

    def _scripted_scenarios(self) -> list[Scenario]:
        return list_scenarios(self._scenario_directory())

    def _scripted_scenario(self, name: str) -> Scenario | None:
        try:
            return load_scenario(name, self._scenario_directory())
        except ScenarioError as exc:
            message = f"Cannot load robot-eval scenario {name!r}: {exc}"
            self._set_scripted_motion_error(message)
            self.logger.error(message)
            return None

    def _circle_duration_estimate(
        self, scenario: Scenario, speed_scale: float
    ) -> float | None:
        """Circle-plan runtime for the recording timer, or None on error.

        Reads the plan JSONs the node will replay; failing here (before
        the arm resets and the node launches) surfaces a missing or
        malformed plan as a UI error instead of a dead node process.
        Mirrors joint_master.py's circle timeline: playback stretched
        by 1/min(1, speed_scale), plus a ramp-in/settle allowance so
        the recording stops after, never before, the trajectory ends.
        """
        cfg = self.launch_cfg.scripted_motion
        params = scenario.parameters
        paths = []
        if cfg.publish_left:
            paths.append(str(params["circle_trajectory_left"]))
        if cfg.publish_right:
            paths.append(str(params["circle_trajectory_right"]))
        play_s = 0.0
        for path in paths:
            try:
                with open(path, encoding="utf-8") as fh:
                    traj = json.load(fh)
                play_s = max(play_s, float(traj["t"][-1]))
            except (OSError, ValueError, KeyError, IndexError) as e:
                message = (
                    f"Circle trajectory {path!r} is missing or "
                    f"malformed ({e}). Plan it first: "
                    "robot-eval plan circle --side left|right"
                )
                self._set_scripted_motion_error(message)
                self.logger.error(message)
                return None
        speed = min(1.0, max(0.01, float(speed_scale)))
        return play_s / speed + 15.0

    def _stress_duration_estimate(
        self, scenario: Scenario, speed_scale: float, laps: int
    ) -> float:
        """Approximate waypoint-plan runtime for the recording timer.

        Mirrors joint_master.py's min-jerk segment timing; includes a
        ramp-in allowance and a safety margin so the recording stops
        after, never before, the trajectory ends.
        """
        params = scenario.parameters
        wl = params["waypoints_left"]
        wr = params["waypoints_right"]
        n = min(len(wl), len(wr))
        if n < 2:
            return 30.0
        v = max(
            0.05,
            float(params["peak_velocity"]) * max(0.01, speed_scale),
        )
        a = max(
            0.1,
            float(params["max_accel"]) * max(0.01, speed_scale),
        )

        def seg(pa, pb, qa, qb):
            dq = max(
                max(abs(pb[j] - pa[j]) for j in range(6)),
                max(abs(qb[j] - qa[j]) for j in range(6)),
            )
            if dq < 1e-6:
                return 0.0
            return max(0.3, 1.875 * dq / v, (5.7735 * dq / a) ** 0.5)

        dwell_count = n - len(
            {
                int(i)
                for i in params["pass_through_indices"]
                if 1 <= int(i) <= n
            }
        )
        total = 8.0  # ramp-in allowance
        for _ in range(max(1, int(laps))):
            for k in range(1, n):
                total += seg(wl[k - 1], wl[k], wr[k - 1], wr[k])
            for k in range(n - 2, -1, -1):
                total += seg(wl[k + 1], wl[k], wr[k + 1], wr[k])
            total += float(params["settle_s"]) * 2 * dwell_count
        return total + 5.0

    def _start_scripted_motion_callback(
        self,
        duration_s: float,
        amplitude_scale: float,
        frequency_scale: float,
        record_motion: bool,
        motion_type: str = "sine",
        speed_scale: float = 0.3,
        laps: int = 1,
    ):
        if self._is_scripted_motion_running():
            self.logger.warning("Scripted motion is already running.")
            return
        if self._any_calibration_running():
            self.logger.warning(
                "A calibration is running; not starting scripted motion."
            )
            return

        # Button callbacks run BEFORE the script body, so an auto-stop
        # timer that came due since the last rerender has not fired
        # yet: is_recording is stale and this run would silently reuse
        # the previous, already-finished episode. Fire the due stop
        # first.
        due_stop_at = st.session_state.get(
            "scripted_motion_recording_auto_stop_at"
        )
        due_stop_uri = st.session_state.get(
            "scripted_motion_recording_auto_stop_uri"
        )
        if (
            due_stop_at is not None
            and due_stop_uri is not None
            and time.monotonic() >= float(due_stop_at)
        ):
            self._auto_stop_scripted_motion_recording(
                expected_data_uri=due_stop_uri
            )
            self._clear_scripted_recording_refresh()

        scenario = self._scripted_scenario(motion_type)
        if scenario is None:
            return
        circle = scenario.mode == "circle"
        circle_estimate: float | None = None
        if circle:
            # Validate the plan files BEFORE the arm resets/moves: a
            # missing plan should surface as a UI error, not a reset
            # followed by a dead node process.
            circle_estimate = self._circle_duration_estimate(
                scenario, speed_scale
            )
            if circle_estimate is None:
                return

        scripted_start_position = self._scripted_motion_reset_position()
        if len(scripted_start_position) != 7:
            self.logger.error(
                "Scripted motion reset/start position must have 7 joint "
                f"values, got {len(scripted_start_position)}."
            )
            return
        # MIT parameters are owned by the bridge launch
        # (piper_dagger_compat.launch.py) and any runtime ros2 param set;
        # scripted motion runs with whatever is currently on the nodes.
        if not self.ros_helper.reset_arm(position=scripted_start_position):
            self.ros_helper.set_control_mode("stop")
            return

        if self.launch_cfg.ros_bridge.disable_inference_service_name:
            if not self.ros_helper.disable_inference():
                self.ros_helper.set_control_mode("stop")
                return

        if not self.ros_helper.set_control_mode("auto"):
            return

        prelaunch_for_recording = (
            record_motion and not self.collecting_state.is_recording
        )
        trigger_file: str | None = None
        ready_file: str | None = None
        min_command_subscribers: int | None = None
        command_subscriber_wait_timeout_s: float | None = None
        if prelaunch_for_recording:
            min_command_subscribers = (
                self.launch_cfg.scripted_motion
                .recording_command_min_subscribers
            )
            command_subscriber_wait_timeout_s = (
                self.launch_cfg.scripted_motion
                .recording_command_subscriber_wait_timeout_s
            )
            sync_dir = Path(tempfile.gettempdir()) / (
                "robo_orchard_scripted_motion"
            )
            sync_dir.mkdir(parents=True, exist_ok=True)
            sync_token = time.monotonic_ns()
            trigger_file = str(sync_dir / f"start_{sync_token}.trigger")
            ready_file = str(sync_dir / f"ready_{sync_token}.ready")
            for sync_file in (trigger_file, ready_file):
                try:
                    os.remove(sync_file)
                except FileNotFoundError:
                    pass

        stress = scenario.mode == "waypoints"
        result_path = (
            "/data/holobrain/robot_eval/"
            f"{scenario.name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        extra_params: dict[str, object] = {
            "scenario": scenario.name,
            "result_path": result_path,
        }
        if self._scenario_directory():
            extra_params["scenario_directory"] = self._scenario_directory()
        node_duration_s = duration_s
        if circle:
            extra_params["speed_scale"] = float(speed_scale)
            # The plan determines its own length; the node ends at plan
            # completion. The recording auto-stop uses the estimate
            # computed before the arm reset.
            node_duration_s = 0.0
            duration_s = float(circle_estimate or 60.0)
        if stress:
            extra_params["speed_scale"] = float(speed_scale)
            extra_params["laps"] = max(1, int(laps))
            # The waypoint plan determines its own length; the node ends
            # at plan completion.
            node_duration_s = 0.0
            # The recording auto-stop needs a duration: use the estimate.
            duration_s = self._stress_duration_estimate(
                scenario, speed_scale, laps
            )

        failures = []
        process = None
        for command in self._scripted_motion_commands(
            duration_s=node_duration_s,
            amplitude_scale=amplitude_scale,
            frequency_scale=frequency_scale,
            start_trigger_file=trigger_file,
            ready_file=ready_file,
            # The waypoint and circle plans ramp in from the actual
            # current pose; the sine sweep starts from the configured
            # reset pose.
            use_current_state=bool(
                scenario.safety.get("use_current_state", False)
            ),
            use_start_position=not bool(
                scenario.safety.get("use_current_state", False)
            ),
            start_position_left=scripted_start_position,
            start_position_right=scripted_start_position,
            min_command_subscribers=min_command_subscribers,
            command_subscriber_wait_timeout_s=(
                command_subscriber_wait_timeout_s
            ),
            extra_params=extra_params,
        ):
            try:
                process = start_process(
                    command,
                    min_live_time=1.0,
                    redirect=True,
                )
                break
            except subprocess.CalledProcessError as e:
                stderr = self._format_process_output(e.stderr)
                stdout = self._format_process_output(e.output)
                detail = stderr or stdout or str(e)
                failures.append((command, detail))
            except FileNotFoundError as e:
                failures.append((command, str(e)))
            except Exception as e:
                failures.append((command, str(e)))

        if process is None:
            details = [
                f"$ {self._format_command(command)}\n{detail}"
                for command, detail in failures
            ]
            error_detail = "Failed to start scripted motion. Tried:\n" + (
                "\n\n".join(details)
            )
            self._set_scripted_motion_error(error_detail)
            self.logger.error(
                "Failed to start scripted motion. See details in the "
                "Scripted Motion section."
            )
            self.ros_helper.set_control_mode("stop")
            return

        self._set_scripted_motion_error(None)
        st.session_state.scripted_motion_process = process
        if trigger_file:
            st.session_state.scripted_motion_trigger_file = trigger_file
        if ready_file:
            st.session_state.scripted_motion_ready_file = ready_file

        recording_to_stop: str | None = None
        if prelaunch_for_recording:
            self.logger.info(
                "Scripted motion process started; waiting for setup readiness "
                "before recording."
            )
            if ready_file and not self._wait_for_scripted_motion_ready(
                ready_file, process
            ):
                self._stop_scripted_motion()
                self.ros_helper.set_control_mode("stop")
                return

            recording_result = self._start_recording_for_scripted_motion()
            if recording_result is None:
                self._stop_scripted_motion()
                self.ros_helper.set_control_mode("stop")
                return
            recording_to_stop, _ = recording_result

            if trigger_file:
                Path(trigger_file).touch()
                self.logger.info(
                    "Recording started; scripted trajectory triggered."
                )
        elif record_motion:
            recording_result = self._start_recording_for_scripted_motion()
            if recording_result is None:
                self._stop_scripted_motion()
                self.ros_helper.set_control_mode("stop")
                return
            recording_to_stop, _ = recording_result

        scheduled_recording_stop = False
        if record_motion and recording_to_stop is not None:
            recording_stop_delay_s = float(duration_s)
            if (
                prelaunch_for_recording
                and command_subscriber_wait_timeout_s is not None
            ):
                recording_stop_delay_s += max(
                    0.0, float(command_subscriber_wait_timeout_s)
                )
            self._schedule_scripted_motion_recording_stop(
                recording_stop_delay_s, recording_to_stop
            )
            scheduled_recording_stop = True
        st.session_state.scripted_motion_last_run = {
            "motion_type": str(motion_type),
            "speed_scale": float(speed_scale),
            "laps": int(laps),
            "duration_s": float(duration_s),
            "amplitude_scale": float(amplitude_scale),
            "frequency_scale": float(frequency_scale),
            "record_motion": bool(record_motion),
            "result_path": result_path,
        }
        self.logger.info("Scripted motion started.")
        if scheduled_recording_stop:
            time.sleep(1.25)
            st.rerun()
