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
from pathlib import Path

import streamlit as st

from robo_orchard_inference_app.utils import stop_process


class CalibrationMixin:
    # --- kp deflection calibration ---
    def _calibration_store_path(self) -> Path:
        repo_root = Path(__file__).resolve().parents[4]
        return repo_root / "calibration" / "deflection_calibrations.json"

    def _load_calibration_store(self) -> dict:
        """The parsed calibration store, or {} when missing/invalid."""
        try:
            return json.loads(self._calibration_store_path().read_text())
        except Exception:
            return {}

    def _calibrated_kps(self) -> dict[str, list[float]]:
        """Side -> sorted calibrated kp values from the deflection store."""
        data = self._load_calibration_store()
        return {
            side: sorted(float(k) for k in entries)
            for side, entries in data.items()
            if side in ("left", "right") and isinstance(entries, dict)
        }

    def _render_follower_kp_calibration_hint(self, follower_kp: float) -> None:
        """Gravity-comp status line under the follower kp field."""
        per_side = self._calibrated_kps()
        if not per_side:
            st.warning(
                "Deflection calibration store not found at "
                f"{self._calibration_store_path()} — the controllers "
                "cannot validate kp."
            )
            return
        common = sorted(
            set(per_side.get("left", [])) & set(per_side.get("right", []))
        )
        kp_list = ", ".join(f"{v:g}" for v in common) if common else "none"
        if any(abs(float(follower_kp) - v) < 1e-6 for v in common):
            st.success(
                "🪶 Gravity compensation is ON — the deflection "
                f"calibration for kp={follower_kp:g} loads automatically "
                f"on both arms. Calibrated kp values: {kp_list}."
            )
        else:
            st.warning(
                f"⚠️ kp={follower_kp:g} has no deflection calibration — "
                "the controllers will reject it on Apply. Use a "
                f"calibrated kp ({kp_list}) or run kp Calibration below."
            )

    # Run calibration scripts in separate process groups.
    def _calibration_process(self, job: str) -> subprocess.Popen | None:
        return st.session_state.get(f"{job}_calibration_process")

    def _is_calibration_running(self, job: str) -> bool:
        process = self._calibration_process(job)
        return process is not None and process.poll() is None

    def _is_kp_calibration_running(self) -> bool:
        return self._is_calibration_running("kp")

    def _any_calibration_running(self) -> bool:
        return self._is_kp_calibration_running()

    def _start_calibration_job(
        self, job: str, script_name: str, cli_args: str, label: str
    ) -> None:
        repo_root = Path(__file__).resolve().parents[4]
        script = repo_root / "calibration" / script_name
        install_setup = repo_root / "ros2_package/install/setup.bash"
        inner = "exec python3 " + shlex.quote(str(script)) + " " + cli_args
        if install_setup.exists():
            inner = f"source {shlex.quote(str(install_setup))} && {inner}"
        log_file = tempfile.NamedTemporaryFile(
            mode="w",
            prefix=f"{job}_calibration_",
            suffix=".log",
            delete=False,
        )
        process = subprocess.Popen(
            ["bash", "-lc", inner],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        log_file.close()
        st.session_state[f"{job}_calibration_process"] = process
        st.session_state[f"{job}_calibration_log"] = log_file.name
        st.session_state[f"{job}_calibration_label"] = label
        self.logger.info(
            f"{job} calibration started ({label}), log: {log_file.name}"
        )

    def _start_kp_calibration(self, side: str, kp: float) -> None:
        self._start_calibration_job(
            "kp",
            "calibrate_kp.py",
            f"--side {side} --kp {kp:g}",
            f"{side} arm, kp={kp:g}",
        )

    def _render_calibration_log(self, log_path: str | None) -> None:
        if not log_path or not os.path.exists(log_path):
            return
        try:
            with open(log_path, errors="replace") as f:
                lines = f.read().splitlines()
        except OSError:
            return
        if lines:
            st.code("\n".join(lines[-12:]), language="text")

    def _clear_calibration_state(self, job: str) -> None:
        log_path = st.session_state.pop(f"{job}_calibration_log", None)
        st.session_state.pop(f"{job}_calibration_process", None)
        st.session_state.pop(f"{job}_calibration_label", None)
        if log_path:
            try:
                os.remove(log_path)
            except OSError:
                pass

    def _render_kp_calibration_panel(self) -> None:
        st.subheader("kp Calibration")
        process = self._calibration_process("kp")
        label = st.session_state.get("kp_calibration_label", "")
        log_path = st.session_state.get("kp_calibration_log")

        if process is not None and process.poll() is None:
            st.info(
                f"Calibrating {label} — the follower is moving through "
                "the measurement grid with compensation OFF (~10 min). "
                "Keep the e-stop in reach."
            )
            self._render_calibration_log(log_path)
            running_cols = st.columns([1, 1])
            with running_cols[0]:
                if st.button(
                    "Refresh status",
                    key=f"{self.key_prefix}_kp_calibration_refresh",
                    use_container_width=True,
                ):
                    st.rerun()
            with running_cols[1]:
                if st.button(
                    "Abort calibration",
                    key=f"{self.key_prefix}_kp_calibration_abort",
                    use_container_width=True,
                ):
                    try:
                        stop_process(process, timeout=10.0)
                    except Exception as e:
                        self.logger.error(
                            f"Failed to stop kp calibration: {e}"
                        )
                    st.rerun()
            return

        if process is not None:
            if process.returncode == 0:
                st.success(
                    f"Calibration finished: {label} is now in the store "
                    "— set that kp and Apply MIT Params to use it."
                )
            else:
                st.error(
                    f"Calibration failed ({label}, "
                    f"exit code {process.returncode})."
                )
            self._render_calibration_log(log_path)
            if st.button(
                "Clear result",
                key=f"{self.key_prefix}_kp_calibration_clear",
                use_container_width=True,
            ):
                self._clear_calibration_state("kp")
                st.rerun()
            return

        st.caption(
            "Measure a new kp's firmware deflection on one arm and add "
            "it to the calibration store (measure + fit, one shot)."
        )
        control_mode = self.collecting_state.inference_state.control_mode
        can_run = (
            control_mode == "stop"
            and not self.collecting_state.is_recording
            and not self._is_scripted_motion_running()
        )
        calib_cols = st.columns([1, 1, 2])
        with calib_cols[0]:
            side = st.selectbox(
                "Arm",
                ("left", "right"),
                key=f"{self.key_prefix}_kp_calibration_side",
            )
        with calib_cols[1]:
            new_kp = st.number_input(
                "New kp",
                min_value=1.0,
                value=25.0,
                step=1.0,
                format="%g",
                key=f"{self.key_prefix}_kp_calibration_kp",
            )
        with calib_cols[2]:
            st.caption("")
            if st.button(
                "Measure & Calibrate",
                key=f"{self.key_prefix}_kp_calibration_start",
                disabled=not can_run,
                use_container_width=True,
            ):
                self._start_kp_calibration(str(side), float(new_kp))
                st.rerun()
        if not can_run:
            st.caption(
                "Requires control mode Stop with no active recording, "
                "scripted motion, or other calibration."
            )

    def _render_calibration_store_summary(self) -> None:
        with st.expander("Calibration store", expanded=False):
            store = self._load_calibration_store()
            if not store:
                st.warning(
                    "Calibration store not found at "
                    f"{self._calibration_store_path()}."
                )
                return
            st.caption(str(self._calibration_store_path()))
            per_side_kps = self._calibrated_kps()
            friction = store.get("friction", {})
            gravity = store.get("gravity", {})
            for side in ("left", "right"):
                st.markdown(f"**{side}**")
                kps = per_side_kps.get(side, [])
                st.caption(
                    "Deflection-calibrated kp values: "
                    + (", ".join(f"{v:g}" for v in kps) if kps else "none")
                )
                fr = friction.get(side)
                if isinstance(fr, dict):
                    scale = fr.get("scale", [])
                    st.caption(
                        "Friction scale (Nm): "
                        + ", ".join(
                            f"J{j + 1}={v:g}" for j, v in enumerate(scale)
                        )
                        + f" | taper {fr.get('taper_velocity', '—')}"
                        + f" | measured {fr.get('measured', '—')}"
                        + f" ({fr.get('method', 'unknown method')})"
                    )
                else:
                    st.caption(
                        "Friction: no store entry (pre-v2 store) — "
                        "controllers use launch/param values."
                    )
                gr = gravity.get(side)
                if isinstance(gr, dict):
                    st.caption(
                        "Gravity scale/joint: "
                        + ", ".join(
                            f"{v:g}" for v in gr.get("scale_per_joint", [])
                        )
                        + f" | measured {gr.get('measured') or '—'}"
                        + f" ({gr.get('method', 'unknown method')})"
                    )
