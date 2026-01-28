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
from dataclasses import dataclass, field

import polling2
import streamlit as st

from robo_orchard_inference_app.components.edit_episode_meta import (
    EditEpisodeMetaComponent,
)
from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.ros_bridge import RosServiceHelper
from robo_orchard_inference_app.ui import StatusConfig, multi_status_indicator


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

    # --- Render State Panel ---
    def _render_state_panel(self):
        """Displays the current configuration and robot state."""
        if self.ros_helper is None:
            return

        with st.expander("ℹ️ Current State", expanded=False):
            control_mode_col, arm_status_col, inference_service_col = (
                st.columns([1, 1, 1])
            )
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
            with arm_status_col:
                multi_status_indicator(
                    current_status=state.arm_ctrl_status,
                    status_config=dict(
                        enabled=StatusConfig(text="Enable", color="green"),
                        disabled=StatusConfig(text="Disable", color="red"),
                    ),
                )
            with inference_service_col:
                multi_status_indicator(
                    current_status=state.is_inference_service_running,
                    status_config={
                        True: StatusConfig(text="推理", color="green"),
                        False: StatusConfig(text="推理", color="grey"),
                    },
                )

    # --- Render Configure Panel ---
    def _render_configure_panel(self):
        with st.expander("📝 Episode Configuration", expanded=True):
            self._configure_panel()

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
                    on_click=self._start_recording_callback,
                    use_container_width=True,
                    help=_get_start_btn_help(),
                    shortcut=self.launch_cfg.ui_control.start_keyboard,
                ):
                    self._handle_start_recording_event()

            with stop_col:
                st.button(
                    "⏹️ Stop",
                    disabled=not self.collecting_state.is_recording,
                    key=f"{self.key_prefix}_stop_record_btn",
                    on_click=self._stop_recording_callback,
                    use_container_width=True,
                    help=_get_stop_btn_help(),
                    shortcut=self.launch_cfg.ui_control.stop_keyboard,
                )

    def _start_recording_callback(self):
        if self.collecting_state.is_recording:
            self.logger.error(
                "An episode is recorded, please decide to save or not first!"  # noqa: E501
            )
            return

        data_uri = self.collecting_state.prepare_recording_path()

        if self.ros_helper.start_recording(uri=data_uri):
            self.logger.info(f"Starting recording for episode: {data_uri}")
            self.collecting_state.at_start_recording()
        else:
            self.logger.error(
                "Failed to start recording! Please check the log panel."
            )

    def _handle_start_recording_event(self):
        """Handles the logic for starting a recording session."""

        if not self.collecting_state.is_recording:
            return

        with st.spinner("Waiting...", show_time=True):
            try:
                # Poll for the __RECORDING__ flag file
                recording_flag = os.path.join(
                    self.collecting_state.current_data_uri, "__RECORDING__"
                )
                polling2.poll(
                    lambda: os.path.exists(recording_flag),
                    timeout=10.0,
                    step=0.1,
                )
                self.logger.info(
                    "Recording started to: "
                    f"{self.collecting_state.current_data_uri}"
                )
            except polling2.TimeoutException:
                self.logger.error(
                    "Failed to start recorder because of timeout"
                )  # noqa: E501
            except Exception as e:
                self.logger.error(
                    "Get unexpected error when handle start "
                    f"recording event: {e}"
                )
            finally:
                st.rerun()

    def _stop_recording_callback(self):
        """Handles the logic for stopping a recording session."""
        if not self.collecting_state.is_recording:
            self.logger.error("Please start recording first!")
            return

        if self.ros_helper.stop_recording():
            self.collecting_state.at_stop_recording()
            self.logger.info(
                "Episode {} saved to: {}".format(
                    self.collecting_state.episode_counter.current(),
                    self.collecting_state.current_data_uri,
                )
            )

        else:
            self.logger.error(
                "Stop recording failed! Please check the log panel."
            )

    # --- Robot Control Panel ---
    def _render_robot_control_panel(self):
        """Renders manual control buttons for the robot."""
        if not self.ros_helper:
            return

        with st.expander("🤖 Robot Control", expanded=True):
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
                    st.button(
                        show_name.capitalize(),
                        on_click=self.ros_helper.set_control_mode,
                        args=(value,),
                        use_container_width=True,
                        key=f"{self.key_prefix}_set_control_mode_{value}",
                    )

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

            # --- Inference service ---
            st.subheader("Inference Control")
            inference_cols = st.columns([1, 1])
            with inference_cols[0]:
                st.button(
                    "Start",
                    key=f"{self.key_prefix}_enable_inference_service",
                    on_click=self.ros_helper.enable_inference,
                    use_container_width=True,
                    args=(self.collecting_state.episode_meta,),
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
                self.ros_helper.disable_arm()
            st.rerun()
        if col2.button("Cancel", use_container_width=True):
            st.rerun()

    def reset_arm_ctrl_callback(self):
        """Resets the robot arm controllers."""
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
        self._render_state_panel()
        self._render_configure_panel()
        self._render_recorder_panel()
        self._render_robot_control_panel()
        self._render_handeye_calib_panel()
