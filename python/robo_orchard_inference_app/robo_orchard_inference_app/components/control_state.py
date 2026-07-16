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

import time
from pathlib import Path

import streamlit as st

from robo_orchard_teleop_ros2.robot_eval import load_scenario


@st.cache_resource
def _ui_prefs() -> dict[str, object]:
    """Preserve operator values when conditional widgets stop rendering."""
    return {}


_UNSERIALIZABLE = object()


class ControlStateMixin:
    @staticmethod
    def _json_safe_value(value):
        if value is None or isinstance(value, bool | int | float | str):
            return value
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, tuple | list):
            items = []
            for item in value:
                safe_item = ControlStateMixin._json_safe_value(item)
                if safe_item is not _UNSERIALIZABLE:
                    items.append(safe_item)
            return items
        if isinstance(value, dict):
            safe_dict = {}
            for key, item in value.items():
                safe_item = ControlStateMixin._json_safe_value(item)
                if safe_item is not _UNSERIALIZABLE:
                    safe_dict[str(key)] = safe_item
            return safe_dict
        if hasattr(value, "model_dump"):
            return ControlStateMixin._json_safe_value(value.model_dump())
        return _UNSERIALIZABLE

    def _session_value(self, key_suffix: str, default):
        return st.session_state.get(f"{self.key_prefix}_{key_suffix}", default)

    def _default_follower_gravity_per_joint(self) -> list[float]:
        mit_cfg = self.launch_cfg.ros_bridge.mit_control
        defaults = list(
            mit_cfg.default_follower_gravity_compensation_scale_per_joint
        )
        if len(defaults) < 6:
            defaults.extend([1.0] * (6 - len(defaults)))
        return [float(v) for v in defaults[:6]]

    def _follower_gravity_per_joint(self) -> list[float]:
        """Per-joint multiplier applied to the rnea gravity torque."""
        defaults = self._default_follower_gravity_per_joint()
        return [
            float(
                self._session_value(f"follower_gravity_pj{j}", defaults[j - 1])
            )
            for j in range(1, 7)
        ]

    def _default_master_gravity_per_joint(self) -> list[float]:
        mit_cfg = self.launch_cfg.ros_bridge.mit_control
        defaults = list(
            mit_cfg.default_master_gravity_compensation_scale_per_joint
        )
        if len(defaults) < 6:
            defaults.extend([1.0] * (6 - len(defaults)))
        return [float(v) for v in defaults[:6]]

    def _master_gravity_per_joint(self) -> list[float]:
        """Per-joint multiplier applied to the master rnea gravity torque."""
        defaults = self._default_master_gravity_per_joint()
        return [
            float(
                self._session_value(f"master_gravity_pj{j}", defaults[j - 1])
            )
            for j in range(1, 7)
        ]

    def _default_master_friction_per_joint(self) -> list[float]:
        mit_cfg = self.launch_cfg.ros_bridge.mit_control
        defaults = list(mit_cfg.default_master_friction_compensation_scale)
        if len(defaults) < 6:
            defaults.extend([0.0] * (6 - len(defaults)))
        return [float(v) for v in defaults[:6]]

    def _master_friction_per_joint(self) -> list[float]:
        """Per-joint base Coulomb friction torque (N·m) for the master."""
        defaults = self._default_master_friction_per_joint()
        return [
            float(
                self._session_value(f"master_friction_pj{j}", defaults[j - 1])
            )
            for j in range(1, 7)
        ]

    def _streamlit_widget_snapshot(self) -> dict[str, object]:
        snapshot: dict[str, object] = {}
        for key, value in st.session_state.items():
            if hasattr(value, "model_dump"):
                continue
            safe_value = self._json_safe_value(value)
            if safe_value is not _UNSERIALIZABLE:
                snapshot[str(key)] = safe_value
        return snapshot

    def _collect_ui_params_snapshot(self) -> dict[str, object]:
        mit_cfg = self.launch_cfg.ros_bridge.mit_control
        scripted_cfg = self.launch_cfg.scripted_motion
        sine_params = load_scenario(
            "sine", scripted_cfg.scenario_directory or None
        ).parameters
        master_gravity_enabled = bool(
            self._session_value(
                "master_gravity_enabled",
                mit_cfg.default_master_gravity_compensation_enabled,
            )
        )
        return {
            "snapshot_version": 1,
            "captured_at_unix_s": time.time(),
            "runtime_state": {
                "control_mode": (
                    self.collecting_state.inference_state.control_mode
                ),
                "arm_ctrl_status": (
                    self.collecting_state.inference_state.arm_ctrl_status
                ),
                "is_inference_service_running": (
                    self.collecting_state.inference_state.is_inference_service_running
                ),
                "is_scripted_motion_running": (
                    self._scripted_motion_process() is not None
                ),
            },
            "mit_control": {
                "master_enabled": (
                    True
                    if master_gravity_enabled
                    else bool(
                        self._session_value(
                            "master_mit_enabled",
                            mit_cfg.default_master_enabled,
                        )
                    )
                ),
                "master_gravity_compensation_enabled": master_gravity_enabled,
                # Gravity compensation floats the master with zero kp and kd.
                "master_kp": (
                    0.0
                    if master_gravity_enabled
                    else float(
                        self._session_value(
                            "master_mit_kp", mit_cfg.default_master_kp
                        )
                    )
                ),
                "master_kd": (
                    0.0
                    if master_gravity_enabled
                    else float(
                        self._session_value(
                            "master_mit_kd", mit_cfg.default_master_kd
                        )
                    )
                ),
                "master_gravity_compensation_urdf_path": (
                    mit_cfg.default_master_gravity_compensation_urdf_path
                ),
                "master_gravity_compensation_scale": float(
                    self._session_value(
                        "master_gravity_scale",
                        mit_cfg.default_master_gravity_compensation_scale,
                    )
                ),
                "master_gravity_compensation_scale_per_joint": (
                    self._master_gravity_per_joint()
                ),
                "master_gravity_compensation_max_t_ref": float(
                    self._session_value(
                        "master_gravity_max_t_ref",
                        mit_cfg.default_master_gravity_compensation_max_t_ref,
                    )
                ),
                "master_friction_compensation_enabled": bool(
                    self._session_value(
                        "master_friction_enabled",
                        mit_cfg.default_master_friction_compensation_enabled,
                    )
                ),
                "master_friction_compensation_scale": (
                    self._master_friction_per_joint()
                ),
                "master_friction_compensation_load_scale": float(
                    self._session_value(
                        "master_friction_load_scale",
                        mit_cfg.default_master_friction_compensation_load_scale,
                    )
                ),
                "master_friction_compensation_min_velocity": float(
                    self._session_value(
                        "master_friction_min_velocity",
                        mit_cfg.default_master_friction_compensation_min_velocity,
                    )
                ),
                "master_friction_compensation_taper_velocity": float(
                    self._session_value(
                        "master_friction_taper_velocity",
                        mit_cfg.default_master_friction_compensation_taper_velocity,
                    )
                ),
                "master_vel_ref": float(
                    self._session_value(
                        "master_mit_vel_ref",
                        mit_cfg.default_master_vel_ref,
                    )
                ),
                "master_torque_ref": float(
                    self._session_value(
                        "master_mit_torque_ref",
                        mit_cfg.default_master_torque_ref,
                    )
                ),
                "follower_enabled": bool(
                    self._session_value(
                        "follower_mit_enabled",
                        mit_cfg.default_follower_enabled,
                    )
                ),
                "follower_kp": float(
                    self._session_value(
                        "follower_mit_kp", mit_cfg.default_follower_kp
                    )
                ),
                "follower_kd": float(
                    self._session_value(
                        "follower_mit_kd", mit_cfg.default_follower_kd
                    )
                ),
                "follower_vel_ref": float(
                    self._session_value(
                        "follower_mit_vel_ref",
                        mit_cfg.default_follower_vel_ref,
                    )
                ),
                "follower_torque_ref": float(
                    self._session_value(
                        "follower_mit_torque_ref",
                        mit_cfg.default_follower_torque_ref,
                    )
                ),
                "follower_velocity_feedforward": bool(
                    self._session_value(
                        "follower_velocity_ff",
                        mit_cfg.default_follower_velocity_feedforward,
                    )
                ),
                "follower_gravity_compensation_enabled": True,
                "follower_gravity_compensation_urdf_path": (
                    mit_cfg.default_follower_gravity_compensation_urdf_path
                ),
                "follower_gravity_compensation_scale": float(
                    self._session_value(
                        "follower_gravity_scale",
                        mit_cfg.default_follower_gravity_compensation_scale,
                    )
                ),
                "follower_gravity_compensation_scale_per_joint": (
                    self._follower_gravity_per_joint()
                ),
                "follower_gravity_compensation_max_t_ref": float(
                    self._session_value(
                        "follower_gravity_max_t_ref",
                        mit_cfg.default_follower_gravity_compensation_max_t_ref,
                    )
                ),
            },
            "scripted_motion": {
                "last_run": st.session_state.get("scripted_motion_last_run"),
                "motion_type": str(
                    self._session_value("scripted_motion_type", "Sinusoidal")
                ),
                "speed_scale": float(
                    self._session_value("stress_speed_scale", 0.3)
                ),
                "laps": int(self._session_value("stress_laps", 1)),
                "duration_s": float(
                    self._session_value(
                        "scripted_duration_s", sine_params["duration_s"]
                    )
                ),
                "record_motion": bool(
                    self._session_value("record_scripted_motion", False)
                ),
                "amplitude_scale": float(
                    self._session_value(
                        "scripted_amplitude_scale",
                        sine_params["amplitude_scale"],
                    )
                ),
                "frequency_scale": float(
                    self._session_value(
                        "scripted_frequency_scale",
                        sine_params["frequency_scale"],
                    )
                ),
                "trajectory_start_position": (
                    self._scripted_motion_reset_position()
                ),
                "uses_current_state_for_start": False,
                "launch_config": self._json_safe_value(scripted_cfg),
                "started_recording": bool(
                    st.session_state.get(
                        "scripted_motion_started_recording", False
                    )
                ),
            },
            "streamlit_session_state": self._streamlit_widget_snapshot(),
        }
