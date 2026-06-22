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

import os
import shlex
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import polling2
import streamlit as st

from robo_orchard_inference_app.components.edit_episode_meta import (
    EditEpisodeMetaComponent,
)
from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.ros_bridge import RosServiceHelper
from robo_orchard_inference_app.ui import StatusConfig, multi_status_indicator
from robo_orchard_inference_app.utils import start_process, stop_process


@dataclass
class MetaRow:
    unique_id: str
    meta_key: str | None = None
    meta_vals: list[str] = field(default_factory=list)


class MainControlComponent(ComponentBase):
    """The main orchestrator component for the control UI.

    This component integrates configuration, recording, and robot control panels,
    managing the overall application state and user workflow.
    """  # noqa: E501

    def __init__(self):
        super().__init__()

        self._configure_panel = EditEpisodeMetaComponent(
            episode_meta=self.collecting_state.episode_meta,
            key_prefix=f"{self.key_prefix}_configure",
        )

        self.ros_helper = RosServiceHelper(
            ros_client=self.ros_client,
            ros_bridge_cfg=self.launch_cfg.ros_bridge,
            inference_state=self.collecting_state.inference_state,
            logger=self.logger,
        )
        self._known_tf_publisher_startup_id: str | None = None

    def _is_tf_publisher_online(self) -> bool:
        return "/static_tf_publisher" in self.ros_helper.get_node_names()

    def _handle_tf_publisher_recovery(self, current_online: bool) -> None:
        if not current_online:
            return

        current_id = self.ros_helper.get_tf_publisher_startup_id()
        if current_id is None:
            return

        if current_id != self._known_tf_publisher_startup_id:
            self.ros_helper.invalidate_static_transform_cache()
            self._known_tf_publisher_startup_id = current_id

    # --- Render State Panel ---
    def _render_state_panel(self):
        """Displays the current configuration and robot state."""
        if self.ros_helper is None:
            return

        with st.expander("ℹ️ Current State", expanded=False):
            control_mode_col, inference_service_col = st.columns([1, 1])
            state = self.collecting_state.inference_state

            with control_mode_col:
                multi_status_indicator(
                    current_status=state.control_mode,
                    status_config=dict(
                        takeover=StatusConfig(text="TakeOver", color="red"),
                        auto=StatusConfig(text="Auto", color="green"),
                        stop=StatusConfig(text="Stop", color="grey"),
                    ),
                )
            with inference_service_col:
                multi_status_indicator(
                    current_status=state.is_inference_service_running,
                    status_config={
                        True: StatusConfig(text="Inference", color="green"),
                        False: StatusConfig(text="Inference", color="grey"),
                    },
                )

    # --- Render Configure Panel ---
    def _render_configure_panel(self):
        with st.expander("📝 Episode Configuration", expanded=True):
            self._configure_panel()
        self._handle_tf_publisher_recovery(self._is_tf_publisher_online())
        self.ros_helper.sync_static_transforms(
            self.collecting_state.episode_meta
        )

    # --- Data Recording Panel ---
    def _render_recorder_panel(self):
        """Renders the data recording controls."""
        if not self.collecting_state.is_configured:
            return

        self.collecting_state.prepare(self.launch_cfg.workspace)

        def _get_start_btn_help() -> str | None:
            if self.launch_cfg.ui_control.start_keyboard is not None:
                start_btn_help = f"Press {self.launch_cfg.ui_control.start_keyboard} to start"  # noqa: E501
            else:
                start_btn_help = None

            return start_btn_help

        def _get_stop_btn_help() -> str | None:
            if self.launch_cfg.ui_control.stop_keyboard is not None:
                stop_btn_help = (
                    f"Press {self.launch_cfg.ui_control.stop_keyboard} to stop"
                )
            else:
                stop_btn_help = None

            return stop_btn_help

        with st.expander("🔴 Data Recorder", expanded=True):
            start_col, stop_col = st.columns(2)

            with start_col:
                if st.button(
                    "▶️ Start",
                    disabled=self.collecting_state.is_recording,
                    key=f"{self.key_prefix}_start_record_btn",
                    use_container_width=True,
                    help=_get_start_btn_help(),
                    shortcut=self.launch_cfg.ui_control.start_keyboard,
                ):
                    self._start_recording_callback()

            with stop_col:
                if st.button(
                    "⏹️ Stop",
                    disabled=not self.collecting_state.is_recording,
                    key=f"{self.key_prefix}_stop_record_btn",
                    use_container_width=True,
                    help=_get_stop_btn_help(),
                    shortcut=self.launch_cfg.ui_control.stop_keyboard,
                ):
                    self._stop_recording_callback()

    def _start_recording_callback(self):
        if self.collecting_state.is_recording:
            self.logger.error(
                "An episode is recorded, please decide to save or not first!"  # noqa: E501
            )
            return

        data_uri = self.collecting_state.prepare_recording_path()
        recording_flag = os.path.join(data_uri, "__RECORDING__")
        start_ok = False

        with st.spinner("Starting recorder...", show_time=True):
            for attempt in range(2):
                if self.ros_helper.start_recording(uri=data_uri):
                    start_ok = True
                    break
                if os.path.exists(recording_flag):
                    start_ok = True
                    break
                if attempt == 0:
                    time.sleep(0.5)

        if not start_ok:
            self.logger.error(
                "Failed to start recording! Please check the log panel."
            )
            return

        self.logger.info(f"Starting recording for episode: {data_uri}")
        self.collecting_state.at_start_recording()
        with st.spinner("Waiting...", show_time=True):
            try:
                polling2.poll(
                    lambda: os.path.exists(recording_flag),
                    timeout=10.0,
                    step=0.1,
                )
                self.logger.info(f"Recording started to: {data_uri}")
            except polling2.TimeoutException:
                self.logger.error(
                    "Failed to start recorder because of timeout"
                )
            except Exception as e:
                self.logger.error(
                    "Get unexpected error when handle start "
                    f"recording event: {e}"
                )
            finally:
                st.rerun()

    def _clear_scripted_recording_refresh(self) -> None:
        for key in (
            "scripted_motion_recording_auto_stop_uri",
            "scripted_motion_recording_auto_stop_at",
            "scripted_motion_recording_refresh_until",
        ):
            st.session_state.pop(key, None)

    def _render_scripted_recording_refresh(self) -> None:
        expected_uri = st.session_state.get(
            "scripted_motion_recording_auto_stop_uri"
        )
        auto_stop_at = st.session_state.get(
            "scripted_motion_recording_auto_stop_at"
        )
        refresh_until = st.session_state.get(
            "scripted_motion_recording_refresh_until"
        )
        if (
            expected_uri is None
            or auto_stop_at is None
            or refresh_until is None
        ):
            return

        active_episode = (
            self.collecting_state.is_recording
            and self.collecting_state.current_data_uri == expected_uri
        )
        if not active_episode:
            self._clear_scripted_recording_refresh()
            return

        now = time.monotonic()
        if now >= float(auto_stop_at):
            self._auto_stop_scripted_motion_recording(
                expected_data_uri=expected_uri
            )
            self._clear_scripted_recording_refresh()
            st.rerun()

        if now > float(refresh_until):
            self._clear_scripted_recording_refresh()
            return

        time.sleep(0.25)
        st.rerun()

    def _start_recording_for_scripted_motion(
        self,
    ) -> tuple[str, bool] | None:
        if self.collecting_state.is_recording:
            if not self.collecting_state.current_data_uri:
                self.logger.error(
                    "Recording is active, but no recording URI is available."
                )
                return None
            st.session_state.scripted_motion_started_recording = False
            return self.collecting_state.current_data_uri, False

        data_uri = self.collecting_state.prepare_recording_path()
        recording_flag = os.path.join(data_uri, "__RECORDING__")
        start_ok = False
        for attempt in range(2):
            if self.ros_helper.start_recording(uri=data_uri):
                start_ok = True
                break
            if os.path.exists(recording_flag):
                start_ok = True
                break
            if attempt == 0:
                time.sleep(0.5)

        if not start_ok:
            self.logger.error(
                "Failed to start scripted motion recording! "
                "Please check the log panel."
            )
            return None

        self.collecting_state.at_start_recording()
        try:
            polling2.poll(
                lambda: os.path.exists(recording_flag),
                timeout=0.5,
                step=0.05,
            )
        except polling2.TimeoutException:
            self.logger.warning(
                "Recorder flag was not observed before scripted motion "
                "start; continuing because the start service succeeded."
            )
        except Exception as e:
            self.logger.warning(
                "Unexpected error while waiting for scripted motion "
                f"recording flag: {e}; continuing because the start "
                "service succeeded."
            )

        st.session_state.scripted_motion_started_recording = True
        self.logger.info(
            "Recording started for scripted motion: "
            f"{self.collecting_state.current_data_uri}"
        )
        return data_uri, True

    def _auto_stop_scripted_motion_recording(
        self, expected_data_uri: str | None = None
    ) -> bool:
        if not self.collecting_state.is_recording:
            return True
        if (
            expected_data_uri is not None
            and self.collecting_state.current_data_uri != expected_data_uri
        ):
            return False

        if self.ros_helper.stop_recording():
            mit_params = self.ros_helper.get_mit_params_snapshot()
            self.collecting_state.at_stop_recording(mit_params=mit_params)
            self.logger.info(
                "Scripted motion recording stopped automatically: "
                f"{self.collecting_state.current_data_uri}"
            )
            return True

        self.logger.error(
            "Auto stop recording failed! Please check the log panel."
        )
        return False

    def _schedule_scripted_motion_recording_stop(
        self, duration_s: float, expected_data_uri: str
    ) -> None:
        delay_s = max(0.0, float(duration_s)) + 0.5
        auto_stop_at = time.monotonic() + delay_s
        st.session_state.scripted_motion_recording_auto_stop_uri = (
            expected_data_uri
        )
        st.session_state.scripted_motion_recording_auto_stop_at = auto_stop_at
        st.session_state.scripted_motion_recording_refresh_until = (
            auto_stop_at + 2.0
        )

    def _stop_recording_callback(self):
        """Handles the logic for stopping a recording session."""
        if not self.collecting_state.is_recording:
            self.logger.error("Please start recording first!")
            return

        if self.ros_helper.stop_recording():
            mit_params = self.ros_helper.get_mit_params_snapshot()
            self.collecting_state.at_stop_recording(mit_params=mit_params)
            st.session_state.scripted_motion_started_recording = False
            self._clear_scripted_recording_refresh()
            self.logger.info(
                "Episode {} saved to: {}".format(
                    self.collecting_state.episode_counter.current(),
                    self.collecting_state.current_data_uri,
                )
            )
            st.rerun()

        else:
            self.logger.error(
                "Stop recording failed! Please check the log panel."
            )

    def _render_mit_control_panel(self) -> dict[str, bool | float]:
        mit_cfg = self.launch_cfg.ros_bridge.mit_control
        st.subheader("MIT Params")

        def _number_input(
            label: str,
            value: float,
            step: float,
            key: str,
            min_value: float | None = None,
        ):
            return st.number_input(
                label,
                min_value=min_value,
                value=float(value),
                step=step,
                format="%.3f",
                key=f"{self.key_prefix}_{key}",
            )

        st.caption("Master")
        master_enabled = st.checkbox(
            "Master MIT mode",
            value=mit_cfg.default_master_enabled,
            key=f"{self.key_prefix}_master_mit_enabled",
        )
        master_cols = st.columns([1, 1, 1, 1])
        with master_cols[0]:
            master_kp = _number_input(
                "Master kp",
                mit_cfg.default_master_kp,
                0.1,
                "master_mit_kp",
                min_value=0.0,
            )
        with master_cols[1]:
            master_kd = _number_input(
                "Master kd",
                mit_cfg.default_master_kd,
                0.01,
                "master_mit_kd",
                min_value=0.0,
            )
        with master_cols[2]:
            master_vel_ref = _number_input(
                "Master vel_ref",
                mit_cfg.default_master_vel_ref,
                0.1,
                "master_mit_vel_ref",
            )
        with master_cols[3]:
            master_torque_ref = _number_input(
                "Master torque_ref",
                mit_cfg.default_master_torque_ref,
                0.1,
                "master_mit_torque_ref",
            )

        st.caption("Follower")
        follower_enabled = st.checkbox(
            "Follower MIT mode",
            value=mit_cfg.default_follower_enabled,
            key=f"{self.key_prefix}_follower_mit_enabled",
        )
        follower_cols = st.columns([1, 1, 1, 1])
        with follower_cols[0]:
            follower_kp = _number_input(
                "Follower kp",
                mit_cfg.default_follower_kp,
                0.1,
                "follower_mit_kp",
                min_value=0.0,
            )
        with follower_cols[1]:
            follower_kd = _number_input(
                "Follower kd",
                mit_cfg.default_follower_kd,
                0.01,
                "follower_mit_kd",
                min_value=0.0,
            )
        with follower_cols[2]:
            follower_vel_ref = _number_input(
                "Follower vel_ref",
                mit_cfg.default_follower_vel_ref,
                0.1,
                "follower_mit_vel_ref",
            )
        with follower_cols[3]:
            follower_torque_ref = _number_input(
                "Follower torque_ref",
                mit_cfg.default_follower_torque_ref,
                0.1,
                "follower_mit_torque_ref",
            )

        return {
            "master_enabled": master_enabled,
            "master_kp": master_kp,
            "master_kd": master_kd,
            "master_vel_ref": master_vel_ref,
            "master_torque_ref": master_torque_ref,
            "follower_enabled": follower_enabled,
            "follower_kp": follower_kp,
            "follower_kd": follower_kd,
            "follower_vel_ref": follower_vel_ref,
            "follower_torque_ref": follower_torque_ref,
        }

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
        self._clear_scripted_motion_trigger_file()
        return None

    def _clear_scripted_motion_trigger_file(self) -> None:
        trigger_file = st.session_state.pop(
            "scripted_motion_trigger_file", None
        )
        if not trigger_file:
            return
        try:
            os.remove(trigger_file)
        except FileNotFoundError:
            pass
        except Exception as e:
            self.logger.warning(
                f"Failed to remove scripted motion trigger file: {e}"
            )

    def _is_scripted_motion_running(self) -> bool:
        return self._scripted_motion_process() is not None

    @staticmethod
    def _scripted_param_value(value: bool | float | str) -> str:
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, float):
            return str(float(value))
        return value

    def _scripted_motion_args(
        self,
        duration_s: float,
        amplitude_scale: float,
        frequency_scale: float,
        start_delay_s: float | None = None,
        start_trigger_file: str | None = None,
    ) -> list[str]:
        cfg = self.launch_cfg.scripted_motion
        params: dict[str, bool | float | str] = {
            "left_command_topic": cfg.left_command_topic,
            "right_command_topic": cfg.right_command_topic,
            "left_state_topic": cfg.left_state_topic,
            "right_state_topic": cfg.right_state_topic,
            "publish_left": cfg.publish_left,
            "publish_right": cfg.publish_right,
            "use_current_state": cfg.use_current_state,
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

        args = ["--ros-args"]
        for name, value in params.items():
            param_value = self._scripted_param_value(value)
            args.extend(["-p", f"{name}:={param_value}"])
        return args

    def _scripted_motion_base_commands(self) -> list[list[str]]:
        cfg = self.launch_cfg.scripted_motion
        candidates: list[list[str]] = []
        if cfg.command:
            candidates.append(list(cfg.command))

        repo_root = Path(__file__).resolve().parents[4]
        script_path = repo_root / (
            "ros2_package/robo_orchard_teleop_ros2/"
            "robo_orchard_teleop_ros2/scripted/joint_master.py"
        )
        if script_path.exists():
            candidates.append(["python3", str(script_path)])

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
    ) -> list[list[str]]:
        args = self._scripted_motion_args(
            duration_s=duration_s,
            amplitude_scale=amplitude_scale,
            frequency_scale=frequency_scale,
            start_delay_s=start_delay_s,
            start_trigger_file=start_trigger_file,
        )
        return [base + args for base in self._scripted_motion_base_commands()]

    def _stop_scripted_motion(self) -> bool:
        process = self._scripted_motion_process()
        if process is None:
            st.session_state.scripted_motion_process = None
            self._clear_scripted_motion_trigger_file()
            return True

        try:
            stop_process(process, timeout=5.0)
        except Exception as e:
            self.logger.error(f"Failed to stop scripted motion: {e}")
            return False

        st.session_state.scripted_motion_process = None
        self._clear_scripted_motion_trigger_file()
        self._set_scripted_motion_error(None)
        self.logger.info("Scripted motion stopped.")
        return True

    def _stop_scripted_motion_and_robot(self) -> None:
        self.ros_helper.set_control_mode("stop")
        self._stop_scripted_motion()
        self.ros_helper.set_control_mode("stop")

    def _stop_scripted_motion_callback(self):
        self._stop_scripted_motion_and_robot()

    def _start_scripted_motion_callback(
        self,
        mit_params: dict[str, bool | float],
        duration_s: float,
        amplitude_scale: float,
        frequency_scale: float,
        record_motion: bool,
    ):
        if self._is_scripted_motion_running():
            self.logger.warning("Scripted motion is already running.")
            return

        if self.collecting_state.is_recording:
            self.logger.warning(
                "Recording is active; scripted motion will use the MIT "
                "params already applied on the robot."
            )
        elif not self.ros_helper.set_mit_params(**mit_params):
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
        setup_delay_s = 0.0
        if prelaunch_for_recording:
            setup_delay_s = max(
                0.0, float(self.launch_cfg.scripted_motion.start_delay_s)
            )
            trigger_dir = Path(tempfile.gettempdir()) / (
                "robo_orchard_scripted_motion"
            )
            trigger_dir.mkdir(parents=True, exist_ok=True)
            trigger_file = str(
                trigger_dir / f"start_{time.monotonic_ns()}.trigger"
            )
            try:
                os.remove(trigger_file)
            except FileNotFoundError:
                pass

        setup_started_at = time.monotonic()
        failures = []
        process = None
        for command in self._scripted_motion_commands(
            duration_s=duration_s,
            amplitude_scale=amplitude_scale,
            frequency_scale=frequency_scale,
            start_delay_s=0.0 if trigger_file else None,
            start_trigger_file=trigger_file,
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

        recording_to_stop: str | None = None
        if prelaunch_for_recording:
            remaining_setup_s = max(
                0.0, setup_delay_s - (time.monotonic() - setup_started_at)
            )
            if remaining_setup_s > 0.0:
                self.logger.info(
                    "Scripted motion process started; waiting "
                    f"{remaining_setup_s:.2f}s more before recording."
                )
                time.sleep(remaining_setup_s)

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
            self._schedule_scripted_motion_recording_stop(
                duration_s, recording_to_stop
            )
            scheduled_recording_stop = True
        self.logger.info("Scripted motion started.")
        if scheduled_recording_stop:
            time.sleep(1.25)
            st.rerun()

    # --- Robot Control Panel ---
    def _render_robot_control_panel(self):
        """Renders manual control buttons for the robot."""
        if not self.ros_helper:
            return

        with st.expander("🤖 Robot Control", expanded=True):
            mit_params = self._render_mit_control_panel()
            st.button(
                "Apply MIT Params",
                key=f"{self.key_prefix}_apply_mit_params",
                disabled=self.collecting_state.is_recording,
                on_click=self.ros_helper.set_mit_params,
                kwargs=mit_params,
                use_container_width=True,
            )

            # --- Control Mode ---
            st.subheader("Control Mode")
            mode_cols = st.columns([1, 1, 1])
            modes = [
                ("takeover", "takeover"),
                ("auto", "auto"),
                ("stop", "stop"),
            ]
            for (
                col,
                (show_name, value),
            ) in zip(mode_cols, modes, strict=False):
                with col:
                    if st.button(
                        show_name.capitalize(),
                        use_container_width=True,
                        key=f"{self.key_prefix}_set_control_mode_{value}",
                    ):
                        if value == "stop":
                            self._stop_scripted_motion_and_robot()
                        elif value == "takeover":
                            self._stop_scripted_motion()
                            self.ros_helper.set_control_mode(
                                value, mit_params=mit_params
                            )
                        else:
                            self.ros_helper.set_control_mode(value)

            # --- Arm Control ---
            st.subheader("Arm Control")
            arm_cols = st.columns([1, 1, 1])
            with arm_cols[0]:
                if st.button(
                    "Enable",
                    key=f"{self.key_prefix}_enable_arm_ctrl",
                    disabled=self.collecting_state.is_recording,
                    use_container_width=True,
                ):
                    self.change_arm_ctrl_dialog(enable=True)
            with arm_cols[1]:
                if st.button(
                    "Disable",
                    key=f"{self.key_prefix}_disable_arm_ctrl",
                    disabled=self.collecting_state.is_recording,
                    use_container_width=True,
                ):
                    self.change_arm_ctrl_dialog(enable=False)
            with arm_cols[2]:
                if st.button(
                    "Reset",
                    key=f"{self.key_prefix}_reset_arm_ctrl",
                    disabled=self.collecting_state.is_recording,
                    use_container_width=True,
                ):
                    self.reset_arm_ctrl_callback()

            # --- Scripted motion ---
            st.subheader("Scripted Motion")
            scripted_cfg = self.launch_cfg.scripted_motion
            scripted_running = self._is_scripted_motion_running()
            motion_settings_cols = st.columns([1, 1])
            with motion_settings_cols[0]:
                scripted_duration_s = st.number_input(
                    "Duration s",
                    min_value=1.0,
                    value=float(scripted_cfg.duration_s),
                    step=1.0,
                    format="%.1f",
                    disabled=scripted_running,
                    key=f"{self.key_prefix}_scripted_duration_s",
                )
            with motion_settings_cols[1]:
                record_scripted_motion = st.checkbox(
                    "Record motion",
                    value=False,
                    disabled=scripted_running,
                    key=f"{self.key_prefix}_record_scripted_motion",
                )

            motion_shape_cols = st.columns([1, 1])
            with motion_shape_cols[0]:
                scripted_amplitude_scale = st.slider(
                    "Amplitude scale",
                    min_value=1.0,
                    max_value=5.0,
                    value=max(1.0, float(scripted_cfg.amplitude_scale)),
                    step=0.05,
                    disabled=scripted_running,
                    key=f"{self.key_prefix}_scripted_amplitude_scale",
                )
            with motion_shape_cols[1]:
                scripted_frequency_scale = st.slider(
                    "Frequency scale",
                    min_value=0.25,
                    max_value=3.0,
                    value=float(scripted_cfg.frequency_scale),
                    step=0.05,
                    disabled=scripted_running,
                    key=f"{self.key_prefix}_scripted_frequency_scale",
                )

            motion_cols = st.columns([1, 1])
            with motion_cols[0]:
                if st.button(
                    "Run Scripted Motion",
                    key=f"{self.key_prefix}_run_scripted_motion",
                    disabled=scripted_running,
                    use_container_width=True,
                ):
                    self._start_scripted_motion_callback(
                        mit_params=mit_params,
                        duration_s=scripted_duration_s,
                        amplitude_scale=scripted_amplitude_scale,
                        frequency_scale=scripted_frequency_scale,
                        record_motion=record_scripted_motion,
                    )
            with motion_cols[1]:
                if st.button(
                    "Stop Scripted Motion",
                    key=f"{self.key_prefix}_stop_scripted_motion",
                    use_container_width=True,
                ):
                    self._stop_scripted_motion_callback()

            self._render_scripted_motion_error()

            # --- Inference service ---
            st.subheader("Inference Control")
            inference_cols = st.columns([1, 1, 1])
            with inference_cols[0]:
                if st.button(
                    "Start",
                    key=f"{self.key_prefix}_enable_inference_service",
                    use_container_width=True,
                ):
                    self._stop_scripted_motion()
                    self.ros_helper.enable_inference(
                        self.collecting_state.episode_meta
                    )

            with inference_cols[1]:
                st.button(
                    "Stop",
                    key=f"{self.key_prefix}_disable_inference_service",
                    on_click=self.ros_helper.disable_inference,
                    use_container_width=True,
                )

    @st.dialog("Confirm Arm State Change", dismissible=False)
    def change_arm_ctrl_dialog(self, enable: bool):
        """Confirmation dialog for enabling/disabling the arm."""
        st.warning(
            "Ensure the robot arm is in a safe position before proceeding.",
            icon="⚠️",
        )
        col1, col2 = st.columns(2)
        if col1.button("Continue", use_container_width=True, type="primary"):
            if enable:
                self.ros_helper.enable_arm()
            else:
                self._stop_scripted_motion_and_robot()
                self.ros_helper.disable_arm()
            st.rerun()
        if col2.button("Cancel", use_container_width=True):
            st.rerun()

    def reset_arm_ctrl_callback(self):
        """Resets the robot arm controllers."""
        self._stop_scripted_motion_and_robot()
        self.ros_helper.disable_inference()
        self.ros_helper.reset_arm()

    def _render_handeye_calib_panel(self):
        """Renders the hand-eye calibration controls."""
        if not self.ros_helper:
            return

        with st.expander("🔧 Hand-Eye Calibration Controller", expanded=False):
            st.button(
                label="Record Current Pose",
                type="primary",
                key=f"{self.key_prefix}_record_handeye_calib_btn",
                on_click=self.ros_helper.record_handeye_calib_pose,
                help="Record the current robot pose and camera image for hand-eye calibration.",  # noqa: E501
                use_container_width=True,
            )
            st.button(
                label="Save and Compute Hand-Eye Calibration",
                type="primary",
                key=f"{self.key_prefix}_save_handeye_calib_btn",
                on_click=self.ros_helper.save_and_compute_handeye_calib,
                help="Save the recorded poses and compute the hand-eye calibration.",  # noqa: E501
                use_container_width=True,
            )

    # --- Entry ---
    def __call__(self):
        """Renders the entire main control UI."""
        self._render_scripted_recording_refresh()
        self._render_state_panel()
        self._render_configure_panel()
        self._render_recorder_panel()
        self._render_robot_control_panel()
        self._render_handeye_calib_panel()
