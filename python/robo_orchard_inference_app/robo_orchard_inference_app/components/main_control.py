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


import streamlit as st

from robo_orchard_inference_app.components.calibration import (
    CalibrationMixin,
)
from robo_orchard_inference_app.components.control_state import (
    ControlStateMixin,
    _ui_prefs,
)
from robo_orchard_inference_app.components.edit_episode_meta import (
    EditEpisodeMetaComponent,
)
from robo_orchard_inference_app.components.mit_control import MitControlMixin
from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.components.recording import RecordingMixin
from robo_orchard_inference_app.components.robot_control import (
    RobotControlMixin,
)
from robo_orchard_inference_app.components.scripted_motion import (
    ScriptedMotionMixin,
)
from robo_orchard_inference_app.ros_bridge import RosServiceHelper
from robo_orchard_inference_app.ui import StatusConfig, multi_status_indicator


class MainControlComponent(
    ControlStateMixin,
    RecordingMixin,
    CalibrationMixin,
    MitControlMixin,
    ScriptedMotionMixin,
    RobotControlMixin,
    ComponentBase,
):
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

    def _pref(self, suffix: str, default):
        """Seed value for a persisted widget (see _ui_prefs)."""
        return _ui_prefs().get(suffix, default)

    def _remember_pref(self, suffix: str) -> None:
        """on_change callback: mirror a widget's new value into _ui_prefs."""
        _ui_prefs()[suffix] = st.session_state[f"{self.key_prefix}_{suffix}"]

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
        """Stop command sources, then reset the robot controllers."""
        self._stop_scripted_motion_and_robot()
        if (
            self.ros_helper.is_inference_node_active()
            and not self.ros_helper.disable_inference()
        ):
            self.logger.warning(
                "Reset is blocked: failed to disable inference service."
            )
            return
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
