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

import streamlit as st

from robo_orchard_inference_app.components.control_state import _ui_prefs
from robo_orchard_teleop_ros2.robot_eval import ScenarioError


class RobotControlMixin:
    def _is_reset_disabled(self) -> bool:
        """Return whether reset is unsafe in the current UI state."""
        return (
            self.collecting_state.is_recording
            or self.collecting_state.inference_state.control_mode != "auto"
            or self._any_calibration_running()
        )

    # --- Robot Control Panel ---
    def _render_robot_control_panel(self):
        """Renders manual control buttons for the robot."""
        if not self.ros_helper:
            return

        with st.expander("🤖 Robot Control", expanded=True):
            self._render_mit_control_panel()
            # The ONLY path that writes MIT params to the robot. Takeover,
            # auto, and scripted motion all run with whatever is currently
            # set on the controller nodes. The callback reads the panel
            # from session state at click time — passing render-time
            # values via kwargs sent the PREVIOUS panel state whenever
            # the toggles and the click reached the server in one batch.
            st.button(
                "Apply MIT Params",
                key=f"{self.key_prefix}_apply_mit_params",
                disabled=(
                    self.collecting_state.is_recording
                    or self._any_calibration_running()
                ),
                on_click=self._apply_mit_params,
                use_container_width=True,
            )
            apply_status = _ui_prefs().get("mit_apply_status")
            if isinstance(apply_status, str):
                if apply_status.startswith("✅"):
                    st.caption(apply_status)
                else:
                    st.error(apply_status)
            else:
                st.caption(
                    "No MIT params applied since app start — nodes are "
                    "running launch defaults or earlier runtime values."
                )

            self._render_kp_calibration_panel()
            self._render_calibration_store_summary()

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
                        # The measurement owns the follower joint_cmd
                        # topic while a calibration runs; only Stop
                        # stays available.
                        disabled=(
                            value != "stop" and self._any_calibration_running()
                        ),
                    ):
                        if value == "stop":
                            self._stop_scripted_motion_and_robot()
                        elif value == "takeover":
                            self._stop_scripted_motion()
                            self.ros_helper.set_control_mode(value)
                        else:
                            self.ros_helper.set_control_mode(value)
                        # Refresh controls rendered earlier with the new mode.
                        st.rerun()

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
                    disabled=self._is_reset_disabled(),
                    use_container_width=True,
                ):
                    self.reset_arm_ctrl_callback()

            # --- Scripted motion ---
            st.subheader("Scripted Motion")
            scripted_running = self._is_scripted_motion_running()
            try:
                scripted_scenarios = self._scripted_scenarios()
            except ScenarioError as exc:
                st.error(f"Cannot load robot-eval scenarios: {exc}")
                return
            scenario_by_label = {
                scenario.label: scenario for scenario in scripted_scenarios
            }
            motion_options = tuple(scenario_by_label)
            if not motion_options:
                st.error("No robot-eval scenarios are installed.")
                return
            persisted_type = self._pref("scripted_motion_type", "Sinusoidal")
            motion_type_label = st.selectbox(
                "Motion type",
                motion_options,
                index=(
                    motion_options.index(persisted_type)
                    if persisted_type in motion_options
                    else 0
                ),
                disabled=scripted_running,
                key=f"{self.key_prefix}_scripted_motion_type",
                on_change=self._remember_pref,
                args=("scripted_motion_type",),
                help=(
                    "Sinusoidal: multi-frequency sweep around the start "
                    "pose. Stress test: min-jerk trajectory through the "
                    "teleop-demonstrated waypoints (forward then reversed), "
                    "including a pass-through of full forward extension. "
                    "Start stress runs at low speed scale and watch the "
                    "first pass. Circle: replays the IK-planned "
                    "end-effector circle from the reviewed scenario."
                ),
            )
            selected_scenario = scenario_by_label[motion_type_label]
            stress_selected = selected_scenario.mode == "waypoints"
            circle_selected = selected_scenario.mode == "circle"

            scripted_duration_s = float(
                selected_scenario.parameters.get("duration_s", 0.0)
            )
            scripted_amplitude_scale = float(
                selected_scenario.parameters.get("amplitude_scale", 1.0)
            )
            scripted_frequency_scale = float(
                selected_scenario.parameters.get("frequency_scale", 1.0)
            )
            stress_speed_scale = float(
                selected_scenario.ui.get("speed_default", 0.3)
            )
            stress_laps = int(selected_scenario.ui.get("laps_default", 1))

            motion_settings_cols = st.columns([1, 1])
            with motion_settings_cols[0]:
                if circle_selected:
                    st.caption(
                        "Run length comes from the planned trajectory files."
                    )
                elif stress_selected:
                    stress_laps = int(
                        st.number_input(
                            "Laps",
                            min_value=1,
                            max_value=20,
                            value=int(self._pref("stress_laps", stress_laps)),
                            step=1,
                            disabled=scripted_running,
                            key=f"{self.key_prefix}_stress_laps",
                            on_change=self._remember_pref,
                            args=("stress_laps",),
                        )
                    )
                else:
                    scripted_duration_s = st.number_input(
                        "Duration s",
                        min_value=1.0,
                        value=float(
                            self._pref(
                                "scripted_duration_s",
                                scripted_duration_s,
                            )
                        ),
                        step=1.0,
                        format="%.1f",
                        disabled=scripted_running,
                        key=f"{self.key_prefix}_scripted_duration_s",
                        on_change=self._remember_pref,
                        args=("scripted_duration_s",),
                    )
            with motion_settings_cols[1]:
                record_scripted_motion = st.checkbox(
                    "Record motion",
                    value=bool(
                        self._pref(
                            "record_scripted_motion",
                            bool(
                                selected_scenario.ui.get(
                                    "record_default", False
                                )
                            ),
                        )
                    ),
                    disabled=scripted_running,
                    key=f"{self.key_prefix}_record_scripted_motion",
                    on_change=self._remember_pref,
                    args=("record_scripted_motion",),
                )

            motion_shape_cols = st.columns([1, 1])
            if stress_selected:
                with motion_shape_cols[0]:
                    stress_speed_scale = st.slider(
                        "Speed scale",
                        min_value=float(
                            selected_scenario.ui.get("speed_min", 0.1)
                        ),
                        max_value=float(
                            selected_scenario.ui.get("speed_max", 1.5)
                        ),
                        value=float(
                            self._pref(
                                "stress_speed_scale",
                                stress_speed_scale,
                            )
                        ),
                        step=float(
                            selected_scenario.ui.get("speed_step", 0.05)
                        ),
                        disabled=scripted_running,
                        key=f"{self.key_prefix}_stress_speed_scale",
                        on_change=self._remember_pref,
                        args=("stress_speed_scale",),
                        help=(
                            "Staged bring-up: 0.3 first (verify the "
                            "interpolated paths), then 0.6, then 1.0 "
                            "(= 2.0 rad/s peak joint velocity)."
                        ),
                    )
                with motion_shape_cols[1]:
                    est = self._stress_duration_estimate(
                        selected_scenario,
                        stress_speed_scale,
                        stress_laps,
                    )
                    st.caption(f"Estimated run time: ~{est:.0f} s")
            elif circle_selected:
                with motion_shape_cols[0]:
                    stress_speed_scale = st.slider(
                        "Speed scale",
                        min_value=float(
                            selected_scenario.ui.get("speed_min", 0.1)
                        ),
                        max_value=float(
                            selected_scenario.ui.get("speed_max", 1.0)
                        ),
                        value=float(
                            self._pref(
                                "circle_speed_scale",
                                stress_speed_scale,
                            )
                        ),
                        step=float(
                            selected_scenario.ui.get("speed_step", 0.05)
                        ),
                        disabled=scripted_running,
                        key=f"{self.key_prefix}_circle_speed_scale",
                        on_change=self._remember_pref,
                        args=("circle_speed_scale",),
                        help=(
                            "Playback time-stretch. 1.0 replays the plan "
                            "as validated; lower values slow it down "
                            "(the node never speeds a plan up)."
                        ),
                    )
                with motion_shape_cols[1]:
                    circle_est = self._circle_duration_estimate(
                        selected_scenario, stress_speed_scale
                    )
                    if circle_est is not None:
                        st.caption(f"Estimated run time: ~{circle_est:.0f} s")
                    else:
                        st.warning(
                            "Circle trajectory files not found. Plan them "
                            "with robot-eval plan circle."
                        )
            else:
                with motion_shape_cols[0]:
                    scripted_amplitude_scale = st.slider(
                        "Amplitude scale",
                        min_value=1.0,
                        max_value=10.0,
                        value=float(
                            self._pref(
                                "scripted_amplitude_scale",
                                max(1.0, scripted_amplitude_scale),
                            )
                        ),
                        step=0.05,
                        disabled=scripted_running,
                        key=f"{self.key_prefix}_scripted_amplitude_scale",
                        on_change=self._remember_pref,
                        args=("scripted_amplitude_scale",),
                    )
                with motion_shape_cols[1]:
                    scripted_frequency_scale = st.slider(
                        "Frequency scale",
                        min_value=0.25,
                        max_value=6.0,
                        value=float(
                            self._pref(
                                "scripted_frequency_scale",
                                scripted_frequency_scale,
                            )
                        ),
                        step=0.05,
                        disabled=scripted_running,
                        key=f"{self.key_prefix}_scripted_frequency_scale",
                        on_change=self._remember_pref,
                        args=("scripted_frequency_scale",),
                    )

            cannot_record_scripted_motion = (
                record_scripted_motion
                and not self.collecting_state.is_configured
            )
            if cannot_record_scripted_motion:
                st.warning(
                    "Select an episode user and task before recording "
                    "scripted motion."
                )

            motion_cols = st.columns([1, 1])
            with motion_cols[0]:
                if st.button(
                    "Run Scripted Motion",
                    key=f"{self.key_prefix}_run_scripted_motion",
                    disabled=(
                        scripted_running or cannot_record_scripted_motion
                    ),
                    use_container_width=True,
                ):
                    self._start_scripted_motion_callback(
                        duration_s=scripted_duration_s,
                        amplitude_scale=scripted_amplitude_scale,
                        frequency_scale=scripted_frequency_scale,
                        record_motion=record_scripted_motion,
                        motion_type=selected_scenario.name,
                        speed_scale=stress_speed_scale,
                        laps=stress_laps,
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
