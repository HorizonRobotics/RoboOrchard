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
import os
import time

import polling2
import streamlit as st

from robo_orchard_inference_app.components.control_state import _ui_prefs


class RecordingMixin:
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

    def _set_gravity_record_for_episode(self, active: bool) -> None:
        """Route follower gravity recording into the current episode dir.

        Enabled when an episode starts recording, cleared when it stops, so
        the per-timestep gravity CSVs land next to the episode's mcap.
        """
        if self.ros_helper is None:
            return
        directory = self.collecting_state.current_data_uri if active else None
        try:
            self.ros_helper.set_follower_gravity_record_dir(directory)
        except Exception as e:
            self.logger.error(
                f"Failed to update gravity-compensation recording path: {e}"
            )

    def _start_recording_callback(self):
        if self.collecting_state.is_recording:
            self.logger.error(
                "An episode is recorded, please decide to save or not first!"  # noqa: E501
            )
            return

        if not self.collecting_state.is_configured:
            self.logger.error(
                "Select an episode user and task before recording."
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
        self._set_gravity_record_for_episode(active=True)
        self._snapshot_mit_params_at_start()
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

        if not self.collecting_state.is_configured:
            self.logger.error(
                "Select an episode user and task before recording "
                "scripted motion."
            )
            return None

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

        try:
            polling2.poll(
                lambda: os.path.exists(recording_flag),
                timeout=10.0,
                step=0.1,
            )
        except polling2.TimeoutException:
            self.ros_helper.stop_recording()
            self.logger.error(
                "Recorder did not become ready before scripted motion "
                "start; scripted motion was not triggered."
            )
            return None
        except Exception as e:
            self.ros_helper.stop_recording()
            self.logger.error(
                "Unexpected error while waiting for scripted motion "
                f"recording flag: {e}; scripted motion was not triggered."
            )
            return None

        self.collecting_state.at_start_recording()
        self._set_gravity_record_for_episode(active=True)
        self._snapshot_mit_params_at_start()
        st.session_state.scripted_motion_started_recording = True
        self.logger.info(
            "Recording started for scripted motion: "
            f"{self.collecting_state.current_data_uri}"
        )
        return data_uri, True

    def _snapshot_mit_params_at_start(self) -> None:
        """Capture the authoritative MIT param snapshot for the episode."""
        try:
            _ui_prefs()["mit_params_at_start"] = (
                self.ros_helper.get_mit_params_snapshot()
            )
        except Exception as e:
            self.logger.error(f"MIT param start snapshot failed: {e}")
            _ui_prefs()["mit_params_at_start"] = None

    def _finalize_recording_stop(self) -> None:
        """Close out an episode after the recorder has been stopped."""
        try:
            stop_snapshot: dict | None = (
                self.ros_helper.get_mit_params_snapshot()
            )
        except Exception as e:
            self.logger.error(f"MIT param snapshot failed: {e}")
            stop_snapshot = {"error": f"snapshot failed: {e}"}
        start_snapshot = _ui_prefs().pop("mit_params_at_start", None)
        if isinstance(start_snapshot, dict):
            # The start snapshot is what this episode actually ran with;
            # the stop snapshot usually already holds the NEXT run's
            # config (re-applied during the recording tail) and is kept
            # only for debugging.
            mit_params = start_snapshot
            mit_params["captured_at"] = "recording_start"
            mit_params["at_stop"] = stop_snapshot
        else:
            mit_params = (
                stop_snapshot
                if isinstance(stop_snapshot, dict)
                else {"error": "no snapshot available"}
            )
            mit_params["captured_at"] = "recording_stop_fallback"
        try:
            ui_params = self._collect_ui_params_snapshot()
        except Exception as e:
            self.logger.error(f"UI param snapshot failed: {e}")
            ui_params = {"error": f"snapshot failed: {e}"}
        try:
            self.collecting_state.at_stop_recording(
                mit_params=mit_params, ui_params=ui_params
            )
        except Exception as e:
            # An exception escaping finalize propagates into the
            # streamlit render loop (red exception screen) and can
            # leave the episode open with no episode_meta.json —
            # observed 2026-07-09 11:17. Force the episode closed and
            # keep the app usable; the error stays in the log panel.
            self.logger.error(
                f"Episode finalize failed ({e}); forcing episode "
                "closure. Check the episode directory for a missing "
                "or partial episode_meta.json."
            )
            self.collecting_state.is_recording = False
        finally:
            self._set_gravity_record_for_episode(active=False)

    def _recorder_appears_stopped(self) -> bool:
        """True when the recorder's episode flag file is gone."""
        uri = self.collecting_state.current_data_uri
        return bool(uri) and not os.path.exists(
            os.path.join(uri, "__RECORDING__")
        )

    def _stop_recording_and_finalize(self) -> bool:
        """Stop the recorder and close the episode; unstick if needed."""
        stopped = self.ros_helper.stop_recording()
        if not stopped and self._recorder_appears_stopped():
            self.logger.warning(
                "Recorder was already stopped; finalizing the episode "
                "anyway to recover."
            )
            stopped = True
        if stopped:
            self._finalize_recording_stop()
        return stopped

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

        if self._stop_recording_and_finalize():
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

        if self._stop_recording_and_finalize():
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
