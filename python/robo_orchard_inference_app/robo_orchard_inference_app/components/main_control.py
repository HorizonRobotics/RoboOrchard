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

from dataclasses import dataclass, field

import streamlit as st

from robo_orchard_inference_app.components.edit_episode_meta import (
    EditEpisodeMetaComponent,
)
from robo_orchard_inference_app.components.mixin import ComponentBase
from robo_orchard_inference_app.ros_bridge import RosServiceHelper
from robo_orchard_inference_app.ui import (
    StatusConfig,
    multi_status_indicator,
)


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
        self.ros_helper.start_status_monitor()
        self._pending_stop_session_id: str | None = None
        self._pending_stop_completed: bool = False
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

        self.ros_helper.refresh_runtime_state()
        snapshot = self.ros_helper.status_snapshot("recorder")
        status = (snapshot or {}).get("data")
        with st.expander("ℹ️ Current State", expanded=False):
            control_col, recorder_col, inference_col = st.columns(3)
            state = self.collecting_state.inference_state

            with control_col:
                st.markdown("**Control**")
                multi_status_indicator(
                    current_status=state.control_mode,
                    status_config=dict(
                        takeover=StatusConfig(text="TakeOver", color="red"),
                        auto=StatusConfig(text="Auto", color="green"),
                        stop=StatusConfig(text="Stop", color="grey"),
                    ),
                )
            with recorder_col:
                st.markdown("**Recording**")
                multi_status_indicator(
                    current_status=status,
                    status_config={
                        value: StatusConfig(
                            text=value.capitalize(), color=color
                        )
                        for value, color in (
                            ("idle", "grey"),
                            ("waiting", "orange"),
                            ("recording", "red"),
                            ("finalizing", "orange"),
                            ("completed", "green"),
                            ("failed", "red"),
                        )
                    },
                )
            with inference_col:
                st.markdown("**Inference**")
                multi_status_indicator(
                    current_status=state.is_inference_service_running,
                    status_config={
                        True: StatusConfig(text="Enabled", color="green"),
                        False: StatusConfig(text="Disabled", color="grey"),
                    },
                )
            if self.collecting_state.recording_start_pending:
                st.caption("Start requested; waiting for Recorder status.")
            if self._pending_stop_session_id:
                st.caption(
                    "Waiting for Recorder completion or metadata settlement."
                )
            if snapshot:
                if snapshot.get("destination"):
                    st.caption(snapshot["destination"])
                if status == "failed":
                    st.error(
                        snapshot.get("failure_reason") or "Recording failed"
                    )
                elif status == "waiting":
                    st.caption(
                        "Waiting for: "
                        + ", ".join(snapshot.get("waiting_topics", []))
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
    def _render_recorder_panel(self) -> None:
        """Renders the data recording controls."""
        snapshot = self.ros_helper.status_snapshot("recorder")
        self._finalize_stopped_episode(snapshot)
        status = (snapshot or {}).get("data")
        if self._update_recorder_state(snapshot):
            st.rerun()

        if self.collecting_state.is_configured:
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
                st.button(
                    "▶️ Start",
                    disabled=(
                        not self.collecting_state.is_configured
                        or self.collecting_state.recording_start_pending
                        or self._pending_stop_session_id is not None
                        or status not in {"idle", "completed", "failed"}
                    ),
                    key=f"{self.key_prefix}_start_record_btn",
                    on_click=self._start_recording_callback,
                    use_container_width=True,
                    help=_get_start_btn_help(),
                    shortcut=self.launch_cfg.ui_control.start_keyboard,
                )

            with stop_col:
                st.button(
                    "⏹️ Stop",
                    disabled=status not in {"waiting", "recording"},
                    key=f"{self.key_prefix}_stop_record_btn",
                    on_click=self._stop_recording_callback,
                    use_container_width=True,
                    help=_get_stop_btn_help(),
                    shortcut=self.launch_cfg.ui_control.stop_keyboard,
                )

    def _update_recorder_state(self, snapshot: dict | None) -> bool:
        """Update node state and the Start guard; report state changes."""
        if snapshot is None:
            return False
        state = self.collecting_state
        previous = (state.is_recording, state.recording_start_pending)
        status = snapshot.get("data")
        state.is_recording = status in {"waiting", "recording", "finalizing"}
        if (
            state.recording_start_pending
            and snapshot.get("destination") == state.current_data_uri
            and snapshot.get("session_id")
            and status
            in {
                "idle",
                "waiting",
                "recording",
                "finalizing",
                "completed",
                "failed",
            }
        ):
            state.recording_session_id = snapshot["session_id"]
            state.recording_start_pending = False
        return previous != (state.is_recording, state.recording_start_pending)

    def _start_recording_callback(self) -> None:
        if self.collecting_state.recording_start_pending:
            self.logger.warning("Recorder Start is still awaiting status.")
            return
        snapshot = self.ros_helper.status_snapshot("recorder")
        if not self._finalize_stopped_episode(snapshot):
            self.logger.warning("Previous Recorder Stop is still unsettled.")
            return
        if snapshot is None or snapshot.get("data") not in {
            "idle",
            "completed",
            "failed",
        }:
            self.logger.error("Recorder is busy or its state is unknown.")
            return

        previous_uri = self.collecting_state.current_data_uri
        previous_log_uri = self.collecting_state.current_log_uri
        previous_session_id = self.collecting_state.recording_session_id
        try:
            data_uri = self.collecting_state.prepare_recording_path()
        except FileExistsError as error:
            self.logger.warning(str(error))
            return
        self.collecting_state.recording_session_id = None
        self.collecting_state.recording_start_pending = True
        try:
            confirmed = self.ros_helper.start_recording(uri=data_uri)
        except Exception as error:
            self.logger.error(f"Recorder Start outcome is unknown: {error}")
            confirmed = None
        if confirmed is False:
            self.collecting_state.recording_start_pending = False
            self.collecting_state.current_data_uri = previous_uri
            self.collecting_state.current_log_uri = previous_log_uri
            self.collecting_state.recording_session_id = previous_session_id
            self.logger.error("Recorder Start was rejected or not sent.")
            return
        self._update_recorder_state(
            self.ros_helper.status_snapshot("recorder")
        )
        if confirmed is True:
            self.logger.info(f"Recording session initialized: {data_uri}")
        elif self.collecting_state.recording_start_pending:
            self.logger.error(
                "Recorder Start outcome is unknown; waiting for node status."
            )

    def _stop_recording_callback(self) -> None:
        """Request Stop; only finalize an episode owned by this App."""
        snapshot = self.ros_helper.status_snapshot("recorder") or {}
        if snapshot.get("data") not in {"waiting", "recording"}:
            self.logger.error(
                "Recorder is not stoppable or its state is unknown."
            )
            return

        self._update_recorder_state(snapshot)
        session = None
        already_pending = self._pending_stop_session_id is not None
        if (
            snapshot.get("destination")
            == self.collecting_state.current_data_uri
            and self.collecting_state.recording_session_id
            and snapshot.get("session_id")
            == self.collecting_state.recording_session_id
        ):
            session = (
                self.collecting_state.recording_session_id,
                self.collecting_state.current_data_uri,
            )
            if not already_pending:
                self._pending_stop_session_id = session[0]
                self._pending_stop_completed = False
        try:
            success = self.ros_helper.stop_recording(session=session)
        except Exception as error:
            self.logger.error(f"Recorder Stop outcome is unknown: {error}")
            success = None
        if success is False:
            if not already_pending:
                self._pending_stop_session_id = None
            self.logger.error(
                "Stop recording failed! Please check Recorder status."
            )
        elif success is True:
            self.logger.info("Recorder Stop accepted; awaiting node status.")
        else:
            self.logger.error(
                "Recorder Stop outcome is unknown; waiting for node status."
            )

    def _finalize_stopped_episode(self, snapshot: dict | None) -> bool:
        """Settle an explicit Stop; return whether nothing remains pending.

        Remember confirmed completion until metadata is saved, even if the
        latest node snapshot disappears or moves to another session.
        """
        if not self._pending_stop_session_id:
            return True
        if (
            self._pending_stop_session_id
            != self.collecting_state.recording_session_id
        ):
            self._pending_stop_session_id = None
            self._pending_stop_completed = False
            return True
        if not self._pending_stop_completed:
            result = self.ros_helper.recorder_stop_result(
                self._pending_stop_session_id,
                self.collecting_state.current_data_uri,
            )
            if result is not None:
                snapshot = result
            if snapshot is None:
                return False
            if snapshot.get("session_id") != self._pending_stop_session_id:
                return False
            status = snapshot.get("data")
            if status not in {"completed", "idle", "failed"}:
                return False
            if (
                status != "completed"
                or snapshot.get("destination")
                != self.collecting_state.current_data_uri
            ):
                self._pending_stop_session_id = None
                return True
            self._pending_stop_completed = True

        try:
            self.collecting_state.at_stop_recording()
        except OSError as error:
            self.logger.error(
                f"Episode metadata settlement failed; will retry: {error}"
            )
            return False
        self._pending_stop_session_id = None
        self._pending_stop_completed = False
        return True

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

            # --- Inference service ---
            st.subheader("Inference Control")
            inference_cols = st.columns([1, 1, 1])
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

            with inference_cols[2]:
                st.button(
                    "Reset",
                    key=f"{self.key_prefix}_reset_arm_ctrl",
                    disabled=self._is_reset_disabled(),
                    on_click=self.reset_arm_ctrl_callback,
                    use_container_width=True,
                )

    def _is_reset_disabled(self) -> bool:
        state = self.collecting_state.inference_state
        return self.collecting_state.recording_controls_locked or (
            state.control_mode in {"takeover", "stop"}
        )

    def reset_arm_ctrl_callback(self):
        """Resets the robot arm controllers."""
        if self._is_reset_disabled():
            self.logger.warning(
                "Reset is blocked by recording or the current control mode."
            )
            return
        # Disabling inference gates the reset only when an inference node
        # is running; with none launched nothing can contend with the
        # reset, so skip the gate instead of blocking on a missing service.
        if self.ros_helper.is_inference_node_active():
            if not self.ros_helper.disable_inference():
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
        st.fragment(run_every=1.0)(self._render_state_panel)()
        self._render_configure_panel()
        st.fragment(run_every=1.0)(self._render_recorder_panel)()
        self._render_robot_control_panel()
        self._render_handeye_calib_panel()
