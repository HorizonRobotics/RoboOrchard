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
import time

import streamlit as st

from robo_orchard_inference_app.components.control_state import _ui_prefs


class MitControlMixin:
    def _render_mit_control_panel(self) -> None:
        """Lean MIT panel: comp on/off toggles and kp/kd for both arms."""
        mit_cfg = self.launch_cfg.ros_bridge.mit_control
        st.subheader("MIT Params")

        def _number_input(
            label: str,
            value: float,
            step: float,
            key: str,
            disabled: bool = False,
        ):
            return st.number_input(
                label,
                min_value=0.0,
                value=float(self._pref(key, value)),
                step=step,
                format="%.3f",
                key=f"{self.key_prefix}_{key}",
                disabled=disabled,
                on_change=self._remember_pref,
                args=(key,),
            )

        st.caption("Master")
        st.checkbox(
            "Master MIT mode",
            value=bool(
                self._pref(
                    "master_mit_enabled", mit_cfg.default_master_enabled
                )
            ),
            key=f"{self.key_prefix}_master_mit_enabled",
            on_change=self._remember_pref,
            args=("master_mit_enabled",),
        )
        master_gravity_enabled = st.checkbox(
            "Master gravity compensation",
            value=bool(
                self._pref(
                    "master_gravity_enabled",
                    mit_cfg.default_master_gravity_compensation_enabled,
                )
            ),
            key=f"{self.key_prefix}_master_gravity_enabled",
            on_change=self._remember_pref,
            args=("master_gravity_enabled",),
            help="Make the master arm weightless + backdrivable",
        )
        st.checkbox(
            "Master friction compensation",
            value=bool(
                self._pref(
                    "master_friction_enabled",
                    mit_cfg.default_master_friction_compensation_enabled,
                )
            ),
            key=f"{self.key_prefix}_master_friction_enabled",
            on_change=self._remember_pref,
            args=("master_friction_enabled",),
            help="Make the master arm frictionless",
            disabled=not master_gravity_enabled,
        )
        master_cols = st.columns([1, 1])
        with master_cols[0]:
            _number_input(
                "Master kp",
                mit_cfg.default_master_kp,
                0.1,
                "master_mit_kp",
                disabled=master_gravity_enabled,
            )
        with master_cols[1]:
            _number_input(
                "Master kd",
                mit_cfg.default_master_kd,
                0.01,
                "master_mit_kd",
                disabled=master_gravity_enabled,
            )
        st.caption("Follower")
        st.checkbox(
            "Follower MIT mode",
            value=bool(
                self._pref(
                    "follower_mit_enabled", mit_cfg.default_follower_enabled
                )
            ),
            key=f"{self.key_prefix}_follower_mit_enabled",
            on_change=self._remember_pref,
            args=("follower_mit_enabled",),
        )
        follower_cols = st.columns([1, 1])
        with follower_cols[0]:
            follower_kp = _number_input(
                "Follower kp",
                mit_cfg.default_follower_kp,
                0.1,
                "follower_mit_kp",
            )
        with follower_cols[1]:
            _number_input(
                "Follower kd",
                mit_cfg.default_follower_kd,
                0.01,
                "follower_mit_kd",
            )
        self._render_follower_kp_calibration_hint(follower_kp)

        st.checkbox(
            "Gravity compensation (follower)",
            value=bool(self._pref("follower_gravity_enabled", True)),
            key=f"{self.key_prefix}_follower_gravity_enabled",
            on_change=self._remember_pref,
            args=("follower_gravity_enabled",),
            help=(
                "rnea gravity torque delivered through the kp position "
                "offset using the kp-indexed deflection calibration."
            ),
        )
        st.checkbox(
            "Velocity feedforward (follower)",
            value=bool(
                self._pref(
                    "follower_velocity_ff",
                    mit_cfg.default_follower_velocity_feedforward,
                )
            ),
            key=f"{self.key_prefix}_follower_velocity_ff",
            on_change=self._remember_pref,
            args=("follower_velocity_ff",),
            help=(
                "Feed the commanded joint velocity as v_des so kd damps "
                "velocity error instead of absolute velocity. Off costs "
                "~30 ms of tracking lag."
            ),
        )

    def _mit_params_from_state(
        self,
    ) -> dict[str, bool | float | list[float]]:
        """MIT panel values read from session state at CALL time."""
        mit_cfg = self.launch_cfg.ros_bridge.mit_control

        def state(suffix: str, default):
            key = f"{self.key_prefix}_{suffix}"
            if key in st.session_state:
                return st.session_state[key]
            # Widget state was dropped (session reset / skipped render):
            # prefer the operator's persisted value over the launch default.
            return _ui_prefs().get(suffix, default)

        master_enabled = bool(
            state("master_mit_enabled", mit_cfg.default_master_enabled)
        )
        master_gravity_enabled = bool(
            state(
                "master_gravity_enabled",
                mit_cfg.default_master_gravity_compensation_enabled,
            )
        )
        master_friction_enabled = bool(
            state(
                "master_friction_enabled",
                mit_cfg.default_master_friction_compensation_enabled,
            )
        )
        master_kp = float(state("master_mit_kp", mit_cfg.default_master_kp))
        master_kd = float(state("master_mit_kd", mit_cfg.default_master_kd))
        # Same coupling as the panel: gravity comp floats the master, so
        # MIT mode is forced on and the position/velocity gains to zero.
        if master_gravity_enabled:
            master_enabled = True
            master_kp = 0.0
            master_kd = 0.0

        return {
            "master_enabled": master_enabled,
            "master_kp": master_kp,
            "master_kd": master_kd,
            "master_vel_ref": float(mit_cfg.default_master_vel_ref),
            "master_torque_ref": float(mit_cfg.default_master_torque_ref),
            "master_gravity_compensation_enabled": master_gravity_enabled,
            "master_friction_compensation_enabled": (
                master_friction_enabled and master_gravity_enabled
            ),
            "follower_enabled": bool(
                state("follower_mit_enabled", mit_cfg.default_follower_enabled)
            ),
            "follower_kp": float(
                state("follower_mit_kp", mit_cfg.default_follower_kp)
            ),
            "follower_kd": float(
                state("follower_mit_kd", mit_cfg.default_follower_kd)
            ),
            "follower_vel_ref": float(mit_cfg.default_follower_vel_ref),
            "follower_torque_ref": float(mit_cfg.default_follower_torque_ref),
            "follower_gravity_compensation_enabled": bool(
                # The panel seeds this checkbox with value=True.
                state("follower_gravity_enabled", True)
            ),
            "follower_velocity_feedforward": bool(
                state(
                    "follower_velocity_ff",
                    mit_cfg.default_follower_velocity_feedforward,
                )
            ),
        }

    # Requested follower key -> node parameter name, for apply readback.
    _FOLLOWER_VERIFY_PARAMS = (
        (
            "grav",
            "follower_gravity_compensation_enabled",
            "mit_gravity_compensation_enabled",
        ),
        ("vff", "follower_velocity_feedforward", "mit_velocity_feedforward"),
    )

    def _apply_mit_params(self) -> None:
        """Apply-button callback: send the panel state as of the click.

        The outcome is verified by reading the params back off the
        follower nodes and pinned under the Apply button until the next
        apply — a transient toast alone was missed when an apply failed
        28 s after app boot (rosbridge services not yet listed) and a
        run silently executed the previous config.
        """
        params = self._mit_params_from_state()
        ok = self.ros_helper.set_mit_params(**params)
        flags = " ".join(
            f"{label}={'on' if params[key] else 'off'}"
            for label, key, _ in self._FOLLOWER_VERIFY_PARAMS
        )
        verified = None
        if ok:
            try:
                snap = self.ros_helper.get_mit_params_snapshot()
                followers = snap.get("follower", [])
                verified = bool(followers) and all(
                    entry.get("status") == "confirmed"
                    and all(
                        bool(entry.get("params", {}).get(node_param))
                        == bool(params[key])
                        for _, key, node_param in self._FOLLOWER_VERIFY_PARAMS
                    )
                    for entry in followers
                )
            except Exception as e:
                self.logger.error(f"MIT apply readback failed: {e}")
        stamp = time.strftime("%H:%M:%S")
        if ok and verified:
            status = f"✅ {stamp} applied & verified — follower: {flags}"
            st.toast(status, icon="✅")
        elif ok:
            status = (
                f"⚠️ {stamp} applied but readback NOT verified — "
                f"intended follower: {flags}"
            )
            st.toast(status, icon="⚠️")
        else:
            status = (
                f"❌ {stamp} APPLY FAILED — nodes unchanged; "
                f"intended follower: {flags}"
            )
            st.toast(status, icon="🚨")
        _ui_prefs()["mit_apply_status"] = status
