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

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from geometry_msgs.msg import PoseStamped
from piper_sdk import C_PiperInterface
from scipy.spatial.transform import (
    Rotation as R,  # noqa: N817
)
from sensor_msgs.msg import JointState

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg

MIT_POSITION_REF_MIN = -12.5
MIT_POSITION_REF_MAX = 12.5
MIT_TORQUE_REF_MIN = -8.0
MIT_TORQUE_REF_MAX = 8.0

__all__ = [
    "PiperLossError",
    "create_piper",
    "get_arm_status",
    "get_arm_ctrl_state",
    "get_arm_state",
    "get_arm_ee_pose",
    "joint_control",
    "joint_mit_control",
    "PinocchioGravityCompensator",
    "GravityCompensationCommand",
    "resolve_deflection_calibration",
    "get_enable_flag",
    "enable_arm_ctrl",
    "switch_piper_ctrl_mode",
    "set_ctrl_method",
]


@dataclass(frozen=True)
class GravityCompensationCommand:
    """MIT command additions produced by gravity compensation."""

    torque_refs: list[float]
    position_offsets: list[float]
    desired_torques: list[float]
    kp_offset_torques: list[float]
    residual_torques: list[float]


class GravityCompensationError(RuntimeError):
    pass


def resolve_deflection_calibration(
    calibration_file: str, side: str, kp: float
) -> tuple[list[float], str]:
    """Look up the kp-indexed deflection calibration for one arm.

    The calibration file (see gravity_id/deflection_calibrations.json)
    maps side -> mit_kp -> {offset_stiffness, deflection_table}. The
    firmware stiffness k_eff depends on the sent mit_kp, so entries are
    only valid at their own kp: a kp with no entry is an error listing
    the calibrated values rather than a silent fallback.

    Returns ``(offset_stiffness, deflection_table_json)`` with the table
    serialized to the string form of the
    ``mit_gravity_compensation_deflection_table`` parameter.
    """
    import json

    path = Path(calibration_file).expanduser()
    if not path.exists():
        raise GravityCompensationError(
            f"Deflection calibration file does not exist: {path}"
        )
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise GravityCompensationError(
            f"Deflection calibration file {path} is not valid JSON: {exc}"
        ) from exc
    per_side = data.get(side)
    if not isinstance(per_side, dict):
        raise GravityCompensationError(
            f"Deflection calibration file {path} has no entries for "
            f"side {side!r}."
        )
    available = sorted(float(k) for k in per_side)
    entry = None
    for key, value in per_side.items():
        if abs(float(key) - float(kp)) < 1e-6:
            entry = value
            break
    if entry is None:
        pretty = ", ".join(f"{v:g}" for v in available)
        raise GravityCompensationError(
            f"No deflection calibration for mit_kp={kp:g} (side {side}) "
            f"in {path}. Calibrated kp values: [{pretty}]. Use one of "
            f"those, or measure this kp with gravity_id/"
            f"measure_deflection.py and add it with fit_deflection.py "
            f"--update-calibration."
        )
    stiffness = [float(v) for v in entry.get("offset_stiffness", [])]
    if len(stiffness) != 6:
        raise GravityCompensationError(
            f"Calibration entry {side}/{kp:g} in {path} must have 6 "
            f"offset_stiffness values."
        )
    table = entry.get("deflection_table", {})
    table_json = json.dumps(table, separators=(",", ":")) if table else ""
    return stiffness, table_json


def _clamp_mit_torque_ref(torque_ref: float) -> float:
    return max(
        MIT_TORQUE_REF_MIN, min(MIT_TORQUE_REF_MAX, float(torque_ref))
    )


class PinocchioGravityCompensator:
    def __init__(self) -> None:
        self.enabled = False
        self.urdf_path = ""
        self.joint_names = [f"joint{i}" for i in range(1, 7)]
        self.scale = 1.0
        # Per-joint multiplier applied on top of ``scale``. An empty/short
        # vector is padded with 1.0, which reproduces the original single-scale
        # behaviour.
        self.per_joint_scale = [1.0] * 6
        self.max_abs_t_ref = 8.0
        # If true, torque beyond the MIT t_ref clamp is represented as a
        # position-reference offset. Disable for diagnostics where t_ref-only
        # behavior must be measured.
        self.use_kp_offset = True
        # If false, no compensation torque is sent through the MIT t_ff
        # channel: t_ref stays 0 and the full desired torque is routed
        # through the kp position offset. Useful when the firmware's t_ff
        # handling is suspect (the two channels have been observed to apply
        # different physical torque for the same nominal Nm).
        self.use_t_ref = True
        # Per-joint measured torque-to-deflection stiffness (Nm/rad) used to
        # convert compensation torque into a position offset. The firmware's
        # internal loop makes the joint far stiffer than the MIT kp we send,
        # so using kp overcompensates. 0 (default) falls back to kp.
        self.offset_stiffness = [0.0] * 6
        # Per-joint measured deflection curves for joints whose stiffness is
        # load-dependent (J2 stiffens from ~190 to ~370 Nm/rad between 2.6
        # and 10 Nm). Each entry is a list of (|tau| Nm, |deflection| rad)
        # knots, both strictly increasing, with an implicit (0, 0) origin;
        # odd symmetry handles negative torque. A non-empty table takes
        # precedence over offset_stiffness for that joint.
        self.offset_deflection_tables: list[list[tuple[float, float]]] = [
            [] for _ in range(6)
        ]
        # Master switch for the deflection tables: False falls back to
        # offset_stiffness/kp without clearing the configured table, so the
        # two conversions can be A/B compared live.
        self.use_deflection_table = True
        # Friction compensation (open_manipulator-style): a per-joint torque
        # added in the direction of motion to cancel Coulomb/stiction. The
        # magnitude is load-scaled (grows with |gravity torque|) and
        # velocity-tapered (full just above the deadband, fading to zero by
        # ``friction_taper_velocity``). Disabled by default (all-zero scale).
        self.friction_enabled = False
        self.friction_scale = [0.0] * 6
        self.friction_load_scale = 0.0
        self.friction_min_velocity = 0.02
        self.friction_taper_velocity = 2.0
        # Static friction (stiction) dithering: while a joint is essentially
        # stationary (|velocity| < friction_static_velocity), a per-joint
        # torque of alternating sign is added every cycle. The zero-mean
        # perturbation keeps the mechanism at the edge of breakaway so small
        # commands move it smoothly instead of hitting a stiction dead zone.
        # Disabled by default (all-zero amplitude).
        self.friction_static_scale = [0.0] * 6
        self.friction_static_velocity = 0.05
        # Inertial (acceleration) feedforward: adds the commanded
        # trajectory's dynamic torque, rnea(q, v_cmd, a_cmd) minus the
        # gravity-only rnea(q, 0, 0), so fast direction changes do not have
        # to generate their inertial torque through tracking error against
        # the firmware spring. The delta rides the same t_ref/kp-offset
        # split as gravity. Disabled by default; the per-joint delta is
        # clamped to +/- inertial_max_torque so a noisy double-derivative
        # of the command stream cannot inject large torque spikes.
        self.inertial_enabled = False
        self.inertial_scale = 1.0
        self.inertial_max_torque = 4.0
        self._dither_switch = False
        self._pin = None
        self._np = None
        self._model = None
        self._data = None
        self._neutral_q = None
        self._zero_v = None
        self._zero_a = None
        self._joint_indices: list[tuple[int, int]] = []
        self._model_key: tuple[str, tuple[str, ...]] | None = None
        # Per-timestep recording of the gravity-compensation computation.
        # When a path is set, every compute() call appends one CSV row with
        # raw RNEA, clamped t_ref, kp offset, and residual torque.
        self._record_path = ""
        self._record_file = None
        self._record_count = 0

    def set_record_path(self, path: str) -> None:
        """Enable/disable per-timestep CSV recording of the computation.

        An empty path disables recording. Each row records the joint
        positions, the raw ``pin.rnea`` gravity torque, the per-joint scale,
        the desired torque, clamped MIT t_ref torque, kp-derived position
        offset, and any residual torque not representable through either
        channel.
        """
        path = str(path or "").strip()
        if path == self._record_path and self._record_file is not None:
            return
        self.close_record()
        self._record_path = path
        if not path:
            return
        record_path = Path(path).expanduser()
        record_path.parent.mkdir(parents=True, exist_ok=True)
        # Line-buffered so the file can be tailed live while recording.
        self._record_file = record_path.open("w", buffering=1)
        n = len(self.joint_names) or 6
        header = ["t_unix", "scale", "max_t_ref"]
        header += [f"q{i}" for i in range(1, n + 1)]
        header += [f"rnea{i}" for i in range(1, n + 1)]
        header += [f"per_joint{i}" for i in range(1, n + 1)]
        header += [f"desired_tau{i}" for i in range(1, n + 1)]
        header += [f"applied_tau{i}" for i in range(1, n + 1)]
        header += [f"kp_position_offset{i}" for i in range(1, n + 1)]
        header += [f"kp_offset_tau{i}" for i in range(1, n + 1)]
        header += [f"residual_tau{i}" for i in range(1, n + 1)]
        header += [f"friction_tau{i}" for i in range(1, n + 1)]
        header += [f"inertial_tau{i}" for i in range(1, n + 1)]
        self._record_file.write(",".join(header) + "\n")
        self._record_count = 0

    def close_record(self) -> None:
        if self._record_file is not None:
            try:
                self._record_file.close()
            finally:
                self._record_file = None

    def _record_row(
        self,
        positions: Sequence[float],
        raw_taus: Sequence[float],
        joint_scales: Sequence[float],
        desired_torques: Sequence[float],
        torque_refs: Sequence[float],
        position_offsets: Sequence[float],
        kp_offset_torques: Sequence[float],
        residual_torques: Sequence[float],
        friction_torques: Sequence[float] | None = None,
        inertial_torques: Sequence[float] | None = None,
    ) -> None:
        if self._record_file is None:
            return
        n = len(self.joint_names) or 6

        def _cells(values: Sequence[float]) -> list[str]:
            cells = [f"{float(v):.6g}" for v in list(values)[:n]]
            cells += [""] * (n - len(cells))
            return cells

        row = [
            f"{time.time():.6f}",
            f"{self.scale:.6g}",
            f"{self.max_abs_t_ref:.6g}",
        ]
        row += _cells(positions)
        row += _cells(raw_taus)
        row += _cells(joint_scales)
        row += _cells(desired_torques)
        row += _cells(torque_refs)
        row += _cells(position_offsets)
        row += _cells(kp_offset_torques)
        row += _cells(residual_torques)
        row += _cells(friction_torques or [])
        row += _cells(inertial_torques or [])
        self._record_file.write(",".join(row) + "\n")
        self._record_count += 1

    def configure(
        self,
        *,
        enabled: bool,
        urdf_path: str,
        joint_names: Sequence[str],
        scale: float,
        max_abs_t_ref: float,
        per_joint_scale: Sequence[float] | None = None,
        use_kp_offset: bool = True,
        use_t_ref: bool = True,
        offset_stiffness: Sequence[float] | None = None,
        offset_deflection_table_json: str = "",
        use_deflection_table: bool = True,
        friction_enabled: bool = False,
        friction_scale: Sequence[float] | None = None,
        friction_load_scale: float = 0.0,
        friction_min_velocity: float = 0.02,
        friction_taper_velocity: float = 2.0,
        friction_static_scale: Sequence[float] | None = None,
        friction_static_velocity: float = 0.05,
        inertial_enabled: bool = False,
        inertial_scale: float = 1.0,
        inertial_max_torque: float = 4.0,
    ) -> None:
        old_state = (
            self.enabled,
            self.urdf_path,
            list(self.joint_names),
            self.scale,
            list(self.per_joint_scale),
            self.max_abs_t_ref,
            self.use_kp_offset,
            self.use_t_ref,
            list(self.offset_stiffness),
            [list(t) for t in self.offset_deflection_tables],
            self.use_deflection_table,
            self.friction_enabled,
            list(self.friction_scale),
            self.friction_load_scale,
            self.friction_min_velocity,
            self.friction_taper_velocity,
            list(self.friction_static_scale),
            self.friction_static_velocity,
            self.inertial_enabled,
            self.inertial_scale,
            self.inertial_max_torque,
            self._model,
            self._data,
            self._neutral_q,
            self._zero_v,
            self._zero_a,
            list(self._joint_indices),
            self._model_key,
        )
        try:
            self.enabled = bool(enabled)
            self.urdf_path = str(urdf_path)
            self.joint_names = list(joint_names)
            self.scale = float(scale)
            self.per_joint_scale = self._normalize_per_joint_scale(
                per_joint_scale, len(self.joint_names)
            )
            self.max_abs_t_ref = float(max_abs_t_ref)
            self.use_kp_offset = bool(use_kp_offset)
            self.use_t_ref = bool(use_t_ref)
            self.offset_stiffness = self._normalize_per_joint(
                offset_stiffness, len(self.joint_names), default=0.0
            )
            self.offset_deflection_tables = self._parse_deflection_tables(
                offset_deflection_table_json, len(self.joint_names)
            )
            self.use_deflection_table = bool(use_deflection_table)
            self.friction_enabled = bool(friction_enabled)
            self.friction_scale = self._normalize_per_joint(
                friction_scale, len(self.joint_names), default=0.0
            )
            self.friction_load_scale = float(friction_load_scale)
            self.friction_min_velocity = float(friction_min_velocity)
            self.friction_taper_velocity = float(friction_taper_velocity)
            self.friction_static_scale = self._normalize_per_joint(
                friction_static_scale, len(self.joint_names), default=0.0
            )
            self.friction_static_velocity = float(friction_static_velocity)
            self.inertial_enabled = bool(inertial_enabled)
            self.inertial_scale = float(inertial_scale)
            self.inertial_max_torque = float(inertial_max_torque)
            model_key = (self.urdf_path, tuple(self.joint_names))
            if model_key != self._model_key:
                self._model = None
                self._data = None
                self._neutral_q = None
                self._zero_v = None
                self._zero_a = None
                self._joint_indices = []
                self._model_key = None
            if self.enabled:
                self._ensure_model()
        except Exception:
            (
                self.enabled,
                self.urdf_path,
                self.joint_names,
                self.scale,
                self.per_joint_scale,
                self.max_abs_t_ref,
                self.use_kp_offset,
                self.use_t_ref,
                self.offset_stiffness,
                self.offset_deflection_tables,
                self.use_deflection_table,
                self.friction_enabled,
                self.friction_scale,
                self.friction_load_scale,
                self.friction_min_velocity,
                self.friction_taper_velocity,
                self.friction_static_scale,
                self.friction_static_velocity,
                self.inertial_enabled,
                self.inertial_scale,
                self.inertial_max_torque,
                self._model,
                self._data,
                self._neutral_q,
                self._zero_v,
                self._zero_a,
                self._joint_indices,
                self._model_key,
            ) = old_state
            raise

    @staticmethod
    def _normalize_per_joint_scale(
        per_joint_scale: Sequence[float] | None, joint_count: int
    ) -> list[float]:
        if not per_joint_scale:
            return [1.0] * joint_count
        values = [float(v) for v in per_joint_scale]
        if len(values) < joint_count:
            # Pad with 1.0 so a short vector only overrides the joints given.
            values = values + [1.0] * (joint_count - len(values))
        elif len(values) > joint_count:
            raise GravityCompensationError(
                f"mit_gravity_compensation_scale_per_joint has "
                f"{len(values)} values, expected at most {joint_count}."
            )
        return values

    @staticmethod
    def _normalize_per_joint(
        values: Sequence[float] | None, joint_count: int, *, default: float
    ) -> list[float]:
        if not values:
            return [default] * joint_count
        out = [float(v) for v in values]
        if len(out) < joint_count:
            out = out + [default] * (joint_count - len(out))
        elif len(out) > joint_count:
            raise GravityCompensationError(
                f"per-joint vector has {len(out)} values, "
                f"expected at most {joint_count}."
            )
        return out

    def _friction_torque(
        self, joint_pos: int, velocity: float, gravity_torque: float
    ) -> float:
        """Direction-of-motion friction feedforward for one joint.

        Mirrors open_manipulator's gravity-compensation controller: a Coulomb
        term applied in the direction of travel, scaled up with joint load
        (``1 + |gravity_torque| * friction_load_scale``) and tapered so it is
        full just above ``friction_min_velocity`` and fades to zero by
        ``friction_taper_velocity``. Returns 0 inside the velocity deadband
        (avoids chatter while the joint is held).
        """
        if not self.friction_enabled:
            return 0.0
        base = (
            self.friction_scale[joint_pos]
            if joint_pos < len(self.friction_scale)
            else 0.0
        )
        if base == 0.0:
            return 0.0
        av = abs(float(velocity))
        if av <= self.friction_min_velocity:
            return 0.0
        load_factor = 1.0 + abs(float(gravity_torque)) * self.friction_load_scale
        denom = self.friction_taper_velocity - self.friction_min_velocity
        if denom <= 0.0:
            taper = 1.0
        elif av >= self.friction_taper_velocity:
            taper = 0.0
        else:
            taper = 1.0 - (av - self.friction_min_velocity) / denom
        direction = 1.0 if velocity > 0.0 else -1.0
        return direction * base * load_factor * taper

    def _static_friction_torque(
        self, joint_pos: int, velocity: float
    ) -> float:
        """Stiction dither for one joint.

        While the joint is essentially stationary
        (``|velocity| < friction_static_velocity``), returns a per-joint
        amplitude whose sign alternates every compute cycle
        (``_dither_switch``). The zero-mean perturbation keeps the gearbox at
        the edge of breakaway so small commands move it smoothly instead of
        sticking, then lurching. Complements ``_friction_torque``, which only
        acts once the joint is already moving.
        """
        if not self.friction_enabled:
            return 0.0
        amplitude = (
            self.friction_static_scale[joint_pos]
            if joint_pos < len(self.friction_static_scale)
            else 0.0
        )
        if amplitude == 0.0:
            return 0.0
        if abs(float(velocity)) >= self.friction_static_velocity:
            return 0.0
        return amplitude if self._dither_switch else -amplitude

    @staticmethod
    def _parse_deflection_tables(
        table_json: str, joint_count: int
    ) -> list[list[tuple[float, float]]]:
        """Parse the deflection-table JSON parameter.

        Format: ``{"2": [[2.61, 0.0137], [9.93, 0.0272]], "3": [...]}``
        keyed by 1-based joint number; each value is a list of
        ``[|torque| Nm, |deflection| rad]`` knots, both strictly increasing.
        An empty string disables all tables.
        """
        tables: list[list[tuple[float, float]]] = [
            [] for _ in range(joint_count)
        ]
        text = str(table_json or "").strip()
        if not text:
            return tables
        import json

        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise GravityCompensationError(
                f"deflection table is not valid JSON: {exc}"
            ) from exc
        if not isinstance(data, dict):
            raise GravityCompensationError(
                "deflection table JSON must be an object keyed by joint "
                "number."
            )
        for key, knots in data.items():
            try:
                joint_num = int(key)
            except ValueError as exc:
                raise GravityCompensationError(
                    f"deflection table key {key!r} is not a joint number."
                ) from exc
            if not 1 <= joint_num <= joint_count:
                raise GravityCompensationError(
                    f"deflection table joint {joint_num} out of range "
                    f"1..{joint_count}."
                )
            parsed: list[tuple[float, float]] = []
            for knot in knots:
                if len(knot) != 2:
                    raise GravityCompensationError(
                        f"deflection table joint {joint_num}: each knot "
                        f"must be [torque, deflection]."
                    )
                parsed.append((float(knot[0]), float(knot[1])))
            prev_tau, prev_delta = 0.0, 0.0
            for tau, delta in parsed:
                if tau <= prev_tau or delta <= prev_delta:
                    raise GravityCompensationError(
                        f"deflection table joint {joint_num}: torque and "
                        f"deflection must both be positive and strictly "
                        f"increasing."
                    )
                prev_tau, prev_delta = tau, delta
            tables[joint_num - 1] = parsed
        return tables

    @staticmethod
    def _monotone_tangents(
        table: Sequence[tuple[float, float]],
    ) -> list[tuple[float, float, float]]:
        """Knots (with implicit origin) plus monotone Hermite tangents.

        Tangents are the harmonic mean of adjacent secant slopes (endpoint
        tangents equal the end secants). All secants are positive for a
        validated table, so every tangent/secant ratio is <= 2, which keeps
        the cubic inside the Fritsch-Carlson monotonicity region: the spline
        is strictly increasing and therefore invertible. A cubic through the
        same knots is used instead of straight segments because the slope
        breaks of piecewise-linear interpolation put velocity kinks into the
        position reference as the load sweeps across a knot (felt as bumpy
        motion on the arm).
        """
        pts = [(0.0, 0.0)] + [(float(t), float(d)) for t, d in table]
        n = len(pts) - 1
        secants = [
            (pts[i + 1][1] - pts[i][1]) / (pts[i + 1][0] - pts[i][0])
            for i in range(n)
        ]
        tangents = [secants[0]]
        for i in range(1, n):
            s0, s1 = secants[i - 1], secants[i]
            tangents.append(2.0 * s0 * s1 / (s0 + s1))
        tangents.append(secants[-1])
        return [
            (pts[i][0], pts[i][1], tangents[i]) for i in range(len(pts))
        ]

    @staticmethod
    def _hermite_eval(
        x0: float, y0: float, m0: float,
        x1: float, y1: float, m1: float,
        x: float,
    ) -> float:
        h = x1 - x0
        t = (x - x0) / h
        t2 = t * t
        t3 = t2 * t
        return (
            y0 * (2.0 * t3 - 3.0 * t2 + 1.0)
            + h * m0 * (t3 - 2.0 * t2 + t)
            + y1 * (-2.0 * t3 + 3.0 * t2)
            + h * m1 * (t3 - t2)
        )

    @staticmethod
    def _deflection_from_torque(
        table: Sequence[tuple[float, float]], torque: float
    ) -> float:
        """|torque| -> |deflection|, odd-symmetric monotone-cubic spline.

        Exact at every knot (implicit (0, 0) origin), C1-smooth in between;
        beyond the last knot, extrapolates linearly with the end tangent.
        """
        a = abs(float(torque))
        sign = 1.0 if torque >= 0.0 else -1.0
        knots = PinocchioGravityCompensator._monotone_tangents(table)
        xl, yl, ml = knots[-1]
        if a >= xl:
            return sign * (yl + (a - xl) * ml)
        for i in range(1, len(knots)):
            x1, y1, m1 = knots[i]
            if a <= x1:
                x0, y0, m0 = knots[i - 1]
                return sign * PinocchioGravityCompensator._hermite_eval(
                    x0, y0, m0, x1, y1, m1, a
                )
        return sign * yl

    @staticmethod
    def _torque_from_deflection(
        table: Sequence[tuple[float, float]], deflection: float
    ) -> float:
        """Inverse of ``_deflection_from_torque`` (spline is monotone)."""
        a = abs(float(deflection))
        sign = 1.0 if deflection >= 0.0 else -1.0
        knots = PinocchioGravityCompensator._monotone_tangents(table)
        xl, yl, ml = knots[-1]
        if a >= yl:
            return sign * (xl + (a - yl) / ml)
        for i in range(1, len(knots)):
            x0, y0, m0 = knots[i - 1]
            x1, y1, m1 = knots[i]
            if a <= y1:
                # Bisection on the strictly increasing cubic segment.
                lo, hi = x0, x1
                for _ in range(60):
                    mid = 0.5 * (lo + hi)
                    if PinocchioGravityCompensator._hermite_eval(
                        x0, y0, m0, x1, y1, m1, mid
                    ) < a:
                        lo = mid
                    else:
                        hi = mid
                return sign * 0.5 * (lo + hi)
        return sign * xl

    @staticmethod
    def _split_gravity_torque(
        *,
        position: float,
        desired_torque: float,
        max_abs_t_ref: float,
        kp: float,
        use_kp_offset: bool = True,
        use_t_ref: bool = True,
        offset_stiffness: float = 0.0,
        deflection_table: Sequence[tuple[float, float]] | None = None,
    ) -> tuple[float, float, float, float]:
        torque_ref = float(desired_torque)
        if not use_t_ref:
            torque_ref = 0.0
        elif max_abs_t_ref >= 0.0:
            effective_max_abs_t_ref = min(
                float(max_abs_t_ref), MIT_TORQUE_REF_MAX
            )
            torque_ref = max(
                -effective_max_abs_t_ref,
                min(effective_max_abs_t_ref, torque_ref),
            )

        # The Piper firmware tracks position_ref with its own internal loop,
        # so the joint's real torque-to-deflection compliance is NOT 1/kp.
        # Precedence for the conversion: measured deflection table (handles
        # load-dependent stiffness) > measured scalar stiffness > legacy kp.
        stiffness = (
            float(offset_stiffness) if offset_stiffness > 0.0 else float(kp)
        )
        use_table = bool(deflection_table)
        overflow_torque = float(desired_torque) - torque_ref
        position_offset = 0.0
        kp_offset_torque = 0.0
        if (
            use_kp_offset
            and overflow_torque != 0.0
            and (use_table or stiffness > 0.0)
        ):
            if use_table:
                requested_offset = (
                    PinocchioGravityCompensator._deflection_from_torque(
                        deflection_table, overflow_torque
                    )
                )
            else:
                requested_offset = overflow_torque / stiffness
            if requested_offset > 0.0:
                available_offset = max(
                    0.0, MIT_POSITION_REF_MAX - float(position)
                )
                position_offset = min(requested_offset, available_offset)
            else:
                available_offset = min(
                    0.0, MIT_POSITION_REF_MIN - float(position)
                )
                position_offset = max(requested_offset, available_offset)
            if use_table:
                kp_offset_torque = (
                    PinocchioGravityCompensator._torque_from_deflection(
                        deflection_table, position_offset
                    )
                )
            else:
                kp_offset_torque = position_offset * stiffness

        residual_torque = overflow_torque - kp_offset_torque
        return torque_ref, position_offset, kp_offset_torque, residual_torque

    def _ensure_model(self) -> None:
        if self._model is not None:
            return
        if not self.urdf_path:
            raise GravityCompensationError(
                "mit_gravity_compensation_urdf_path must be set before "
                "enabling gravity compensation."
            )

        try:
            import numpy as np
            import pinocchio as pin
        except ImportError as exc:
            raise GravityCompensationError(
                "Pinocchio is required for MIT gravity compensation. "
                "Install python3-pinocchio or the pin/pinocchio Python "
                "package in the ROS environment."
            ) from exc

        urdf_path = Path(self.urdf_path).expanduser()
        if not urdf_path.exists():
            raise GravityCompensationError(
                f"Gravity compensation URDF does not exist: {urdf_path}"
            )

        model = pin.buildModelFromUrdf(str(urdf_path))
        model.gravity.linear = np.array([0.0, 0.0, -9.81])
        joint_indices: list[tuple[int, int]] = []
        for joint_name in self.joint_names:
            joint_id = model.getJointId(joint_name)
            if joint_id >= model.njoints:
                raise GravityCompensationError(
                    f"Joint {joint_name!r} not found in URDF {urdf_path}"
                )
            joint_model = model.joints[joint_id]
            if joint_model.nq != 1 or joint_model.nv != 1:
                raise GravityCompensationError(
                    f"Joint {joint_name!r} must be a single-DoF joint for "
                    "Piper MIT gravity compensation."
                )
            joint_indices.append((joint_model.idx_q, joint_model.idx_v))

        self._pin = pin
        self._np = np
        self._model = model
        self._data = model.createData()
        self._neutral_q = pin.neutral(model)
        self._zero_v = np.zeros(model.nv)
        self._zero_a = np.zeros(model.nv)
        self._joint_indices = joint_indices
        self._model_key = (str(self.urdf_path), tuple(self.joint_names))

    def compute(
        self,
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
    ) -> list[float]:
        return self.compute_command(
            positions, kp=0.0, velocities=velocities
        ).torque_refs

    def compute_command(
        self,
        positions: Sequence[float],
        *,
        kp: float,
        velocities: Sequence[float] | None = None,
        command_velocities: Sequence[float] | None = None,
        command_accelerations: Sequence[float] | None = None,
    ) -> GravityCompensationCommand:
        joint_count = min(6, len(positions))
        if not self.enabled:
            zeros = [0.0] * joint_count
            return GravityCompensationCommand(
                torque_refs=zeros,
                position_offsets=list(zeros),
                desired_torques=list(zeros),
                kp_offset_torques=list(zeros),
                residual_torques=list(zeros),
            )
        self._ensure_model()
        assert self._pin is not None
        assert self._model is not None
        assert self._data is not None
        assert self._neutral_q is not None
        assert self._zero_v is not None
        assert self._zero_a is not None

        q = self._neutral_q.copy()
        for (idx_q, _), position in zip(
            self._joint_indices, positions, strict=False
        ):
            q[idx_q] = float(position)

        tau = self._pin.rnea(
            self._model, self._data, q, self._zero_v, self._zero_a
        )
        # Inertial feedforward: full inverse dynamics of the commanded
        # trajectory minus the gravity-only torque above. The delta is the
        # inertia + Coriolis torque the motors must produce during
        # commanded accelerations.
        inertial_deltas = [0.0] * joint_count
        if (
            self.inertial_enabled
            and command_accelerations is not None
        ):
            v = self._zero_v.copy()
            a = self._zero_a.copy()
            for joint_pos, (_, idx_v) in enumerate(self._joint_indices):
                if (
                    command_velocities is not None
                    and joint_pos < len(command_velocities)
                ):
                    v[idx_v] = float(command_velocities[joint_pos])
                if joint_pos < len(command_accelerations):
                    a[idx_v] = float(command_accelerations[joint_pos])
            tau_dyn = self._pin.rnea(self._model, self._data, q, v, a)
            clamp = abs(self.inertial_max_torque)
            for joint_pos, (_, idx_v) in enumerate(
                self._joint_indices[:joint_count]
            ):
                delta = (
                    float(tau_dyn[idx_v]) - float(tau[idx_v])
                ) * self.inertial_scale
                inertial_deltas[joint_pos] = max(-clamp, min(clamp, delta))
        torque_refs: list[float] = []
        position_offsets: list[float] = []
        desired_torques: list[float] = []
        kp_offset_torques: list[float] = []
        residual_torques: list[float] = []
        raw_taus: list[float] = []
        joint_scales: list[float] = []
        friction_torques: list[float] = []
        for joint_pos, (_, idx_v) in enumerate(
            self._joint_indices[: len(positions)]
        ):
            joint_scale = (
                self.per_joint_scale[joint_pos]
                if joint_pos < len(self.per_joint_scale)
                else 1.0
            )
            raw_tau = float(tau[idx_v])
            gravity_torque = raw_tau * self.scale * joint_scale
            velocity = (
                float(velocities[joint_pos])
                if velocities is not None and joint_pos < len(velocities)
                else 0.0
            )
            friction_torque = self._friction_torque(
                joint_pos, velocity, gravity_torque
            ) + self._static_friction_torque(joint_pos, velocity)
            # Friction and inertial feedforward ride the same MIT t_ref/kp
            # split + clamp as gravity.
            desired_torque = (
                gravity_torque
                + friction_torque
                + inertial_deltas[joint_pos]
            )
            (
                torque_ref,
                position_offset,
                kp_offset_torque,
                residual_torque,
            ) = self._split_gravity_torque(
                position=float(positions[joint_pos]),
                desired_torque=desired_torque,
                max_abs_t_ref=self.max_abs_t_ref,
                kp=float(kp),
                use_kp_offset=self.use_kp_offset,
                use_t_ref=self.use_t_ref,
                offset_stiffness=(
                    self.offset_stiffness[joint_pos]
                    if joint_pos < len(self.offset_stiffness)
                    else 0.0
                ),
                deflection_table=(
                    self.offset_deflection_tables[joint_pos]
                    if self.use_deflection_table
                    and joint_pos < len(self.offset_deflection_tables)
                    else None
                ),
            )
            friction_torques.append(friction_torque)
            torque_refs.append(torque_ref)
            position_offsets.append(position_offset)
            desired_torques.append(desired_torque)
            kp_offset_torques.append(kp_offset_torque)
            residual_torques.append(residual_torque)
            raw_taus.append(raw_tau)
            joint_scales.append(joint_scale)
        # One phase flip per control cycle (not per joint) so all joints
        # dither in lockstep and each joint's perturbation averages to zero.
        self._dither_switch = not self._dither_switch
        if self._record_file is not None:
            self._record_row(
                positions,
                raw_taus,
                joint_scales,
                desired_torques,
                torque_refs,
                position_offsets,
                kp_offset_torques,
                residual_torques,
                friction_torques,
                inertial_deltas,
            )
        return GravityCompensationCommand(
            torque_refs=torque_refs,
            position_offsets=position_offsets,
            desired_torques=desired_torques,
            kp_offset_torques=kp_offset_torques,
            residual_torques=residual_torques,
        )

global_logger = logging.getLogger(__name__)


class PiperLossError(Exception):
    pass


def create_piper(can_port: str) -> C_PiperInterface:
    piper = C_PiperInterface(can_name=can_port)
    piper.ConnectPort()

    # NOTE: refresh piper message, without this stage,
    # you may get error message
    _ = piper.GetArmStatus()
    _ = get_arm_ctrl_state(piper)
    _ = get_arm_ee_pose(piper)
    _ = get_enable_flag(piper)
    _ = get_arm_status(piper)

    return piper


def get_arm_status(piper: C_PiperInterface) -> PiperStatusMsg:
    status_msg = piper.GetArmStatus()

    arm_status = PiperStatusMsg()
    arm_status.ctrl_mode = status_msg.arm_status.ctrl_mode
    arm_status.arm_status = status_msg.arm_status.arm_status
    arm_status.mode_feedback = status_msg.arm_status.mode_feed
    arm_status.teach_status = status_msg.arm_status.teach_status
    arm_status.motion_status = status_msg.arm_status.motion_status
    arm_status.trajectory_num = status_msg.arm_status.trajectory_num
    arm_status.err_code = status_msg.arm_status.err_code
    arm_status.joint_1_angle_limit = (
        status_msg.arm_status.err_status.joint_1_angle_limit
    )
    arm_status.joint_2_angle_limit = (
        status_msg.arm_status.err_status.joint_2_angle_limit
    )
    arm_status.joint_3_angle_limit = (
        status_msg.arm_status.err_status.joint_3_angle_limit
    )
    arm_status.joint_4_angle_limit = (
        status_msg.arm_status.err_status.joint_4_angle_limit
    )
    arm_status.joint_5_angle_limit = (
        status_msg.arm_status.err_status.joint_5_angle_limit
    )
    arm_status.joint_6_angle_limit = (
        status_msg.arm_status.err_status.joint_6_angle_limit
    )
    arm_status.communication_status_joint_1 = (
        status_msg.arm_status.err_status.communication_status_joint_1
    )  # noqa: E501
    arm_status.communication_status_joint_2 = (
        status_msg.arm_status.err_status.communication_status_joint_2
    )  # noqa: E501
    arm_status.communication_status_joint_3 = (
        status_msg.arm_status.err_status.communication_status_joint_3
    )  # noqa: E501
    arm_status.communication_status_joint_4 = (
        status_msg.arm_status.err_status.communication_status_joint_4
    )  # noqa: E501
    arm_status.communication_status_joint_5 = (
        status_msg.arm_status.err_status.communication_status_joint_5
    )  # noqa: E501
    arm_status.communication_status_joint_6 = (
        status_msg.arm_status.err_status.communication_status_joint_6
    )  # noqa: E501
    return arm_status


def get_arm_ctrl_state(piper: C_PiperInterface) -> JointState:
    joint_states = JointState()
    joint_states.name = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]
    joint_states.position = [0.0] * 7
    joint_states.velocity = [0.0] * 7
    joint_states.effort = [0.0] * 7

    joint_state_factor = 1.0 / 1000 * 0.017444
    gripper_state_factor = 1.0 / 1000000

    joint_msg = piper.GetArmJointCtrl()
    gripper_msg = piper.GetArmGripperCtrl()

    joint_states.position = [
        joint_msg.joint_ctrl.joint_1 * joint_state_factor,
        joint_msg.joint_ctrl.joint_2 * joint_state_factor,
        joint_msg.joint_ctrl.joint_3 * joint_state_factor,
        joint_msg.joint_ctrl.joint_4 * joint_state_factor,
        joint_msg.joint_ctrl.joint_5 * joint_state_factor,
        joint_msg.joint_ctrl.joint_6 * joint_state_factor,
        gripper_msg.gripper_ctrl.grippers_angle * gripper_state_factor,
    ]

    return joint_states


def get_arm_state(piper: C_PiperInterface) -> JointState:
    joint_states = JointState()
    joint_states.name = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]
    joint_states.position = [0.0] * 7
    joint_states.velocity = [0.0] * 7
    joint_states.effort = [0.0] * 7

    joint_msg = piper.GetArmJointMsgs()
    spd_info_msg = piper.GetArmHighSpdInfoMsgs()
    gripper_msg = piper.GetArmGripperMsgs()

    # Here, you can set the joint positions to any value you want
    # The raw data obtained is in degrees multiplied by 1000.
    # To convert to radians, divide by 1000, multiply by π/180,
    # and limit to 5 decimal places
    joint_0: float = (joint_msg.joint_state.joint_1 / 1000) * 0.017444
    joint_1: float = (joint_msg.joint_state.joint_2 / 1000) * 0.017444
    joint_2: float = (joint_msg.joint_state.joint_3 / 1000) * 0.017444
    joint_3: float = (joint_msg.joint_state.joint_4 / 1000) * 0.017444
    joint_4: float = (joint_msg.joint_state.joint_5 / 1000) * 0.017444
    joint_5: float = (joint_msg.joint_state.joint_6 / 1000) * 0.017444
    joint_6: float = gripper_msg.gripper_state.grippers_angle / 1000000

    vel_0: float = spd_info_msg.motor_1.motor_speed / 1000
    vel_1: float = spd_info_msg.motor_2.motor_speed / 1000
    vel_2: float = spd_info_msg.motor_3.motor_speed / 1000
    vel_3: float = spd_info_msg.motor_4.motor_speed / 1000
    vel_4: float = spd_info_msg.motor_5.motor_speed / 1000
    vel_5: float = spd_info_msg.motor_6.motor_speed / 1000

    effort_0: float = spd_info_msg.motor_1.effort / 1000
    effort_1: float = spd_info_msg.motor_2.effort / 1000
    effort_2: float = spd_info_msg.motor_3.effort / 1000
    effort_3: float = spd_info_msg.motor_4.effort / 1000
    effort_4: float = spd_info_msg.motor_5.effort / 1000
    effort_5: float = spd_info_msg.motor_6.effort / 1000
    effort_6: float = gripper_msg.gripper_state.grippers_effort / 1000

    joint_states.position = [
        joint_0,
        joint_1,
        joint_2,
        joint_3,
        joint_4,
        joint_5,
        joint_6,
    ]
    joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
    joint_states.effort = [
        effort_0,
        effort_1,
        effort_2,
        effort_3,
        effort_4,
        effort_5,
        effort_6,
    ]

    return joint_states


def get_arm_ee_pose(piper: C_PiperInterface) -> PoseStamped:
    endpos = PoseStamped()

    pose_msg = piper.GetArmEndPoseMsgs()

    endpos.pose.position.x = pose_msg.end_pose.X_axis / 1000000
    endpos.pose.position.y = pose_msg.end_pose.Y_axis / 1000000
    endpos.pose.position.z = pose_msg.end_pose.Z_axis / 1000000
    roll = pose_msg.end_pose.RX_axis / 1000
    pitch = pose_msg.end_pose.RY_axis / 1000
    yaw = pose_msg.end_pose.RZ_axis / 1000
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    endpos.pose.orientation.x = quaternion[0]
    endpos.pose.orientation.y = quaternion[1]
    endpos.pose.orientation.z = quaternion[2]
    endpos.pose.orientation.w = quaternion[3]

    return endpos


def joint_control(
    piper: C_PiperInterface,
    joint_data: JointState,
    has_gripper: bool = True,
    gripper_val_mutiple: float = 1.0,
):
    factor = 57324.840764  # 1000 * 180 / 3.14

    joint_positions = {}

    gripper = 0

    for idx, joint_name in enumerate(joint_data.name):
        joint_positions[joint_name] = round(joint_data.position[idx] * factor)

    if len(joint_data.position) >= 7:
        gripper = round(joint_data.position[6] * 1000 * 1000)
        gripper = gripper * gripper_val_mutiple

    # control joints
    piper.JointCtrl(
        joint_positions.get("joint1", 0),
        joint_positions.get("joint2", 0),
        joint_positions.get("joint3", 0),
        joint_positions.get("joint4", 0),
        joint_positions.get("joint5", 0),
        joint_positions.get("joint6", 0),
    )

    # control gripper
    if has_gripper:
        piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)


def joint_mit_control(
    piper: C_PiperInterface,
    joint_data: JointState,
    mit_kp: float,
    mit_kd: float,
    mit_torque_ref: float = 0.0,
    gravity_compensator: PinocchioGravityCompensator | None = None,
    has_gripper: bool = False,
    gripper_val_mutiple: float = 1.0,
    velocity_ref: Sequence[float] | None = None,
    measured_velocity: Sequence[float] | None = None,
    command_velocity: Sequence[float] | None = None,
    command_acceleration: Sequence[float] | None = None,
):
    joint_count = min(6, len(joint_data.position))
    gravity_command = GravityCompensationCommand(
        torque_refs=[0.0] * joint_count,
        position_offsets=[0.0] * joint_count,
        desired_torques=[0.0] * joint_count,
        kp_offset_torques=[0.0] * joint_count,
        residual_torques=[0.0] * joint_count,
    )
    if gravity_compensator is not None and gravity_compensator.enabled:
        gravity_command = gravity_compensator.compute_command(
            joint_data.position[:joint_count],
            kp=float(mit_kp),
            velocities=measured_velocity,
            command_velocities=command_velocity,
            command_accelerations=command_acceleration,
        )

    for idx in range(joint_count):
        # v_des feedforward: kd then damps the velocity tracking error
        # (v_des - q_dot) instead of absolute velocity, removing the drag the
        # kd term otherwise applies while following a moving command.
        v_des = (
            float(velocity_ref[idx])
            if velocity_ref is not None and idx < len(velocity_ref)
            else 0.0
        )
        position_ref = float(joint_data.position[idx])
        if idx < len(gravity_command.position_offsets):
            position_ref += gravity_command.position_offsets[idx]
        position_ref = max(
            MIT_POSITION_REF_MIN, min(MIT_POSITION_REF_MAX, position_ref)
        )
        gravity_torque_ref = (
            gravity_command.torque_refs[idx]
            if idx < len(gravity_command.torque_refs)
            else 0.0
        )
        piper.JointMitCtrl(
            idx + 1,
            position_ref,
            v_des,
            mit_kp,
            mit_kd,
            _clamp_mit_torque_ref(float(mit_torque_ref) + gravity_torque_ref),
        )

    # The gripper is not a MIT joint; keep driving it the same way as
    # joint_control does.
    if has_gripper:
        gripper = 0
        if len(joint_data.position) >= 7:
            gripper = round(joint_data.position[6] * 1000 * 1000)
            gripper = gripper * gripper_val_mutiple
        piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)


def get_enable_flag(piper: C_PiperInterface):
    msg = piper.GetArmLowSpdInfoMsgs()

    enable_flag = (
        msg.motor_1.foc_status.driver_enable_status
        and msg.motor_2.foc_status.driver_enable_status
        and msg.motor_3.foc_status.driver_enable_status
        and msg.motor_4.foc_status.driver_enable_status
        and msg.motor_5.foc_status.driver_enable_status
        and msg.motor_6.foc_status.driver_enable_status
    )
    return enable_flag


def enable_arm_ctrl(
    piper: C_PiperInterface,
    timeout: float = 5,
):
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time

        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)

        flag = get_enable_flag(piper)

        if flag:
            return

        if elapsed_time > timeout:
            break

        time.sleep(1)

    raise TimeoutError


def switch_piper_ctrl_mode(
    piper: C_PiperInterface,
    target_mode: int,
    is_mit: bool,
    timeout: float = 5,
):
    start_time = time.time()

    while True:
        if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:  # noqa: E501
            return

        elapsed_time = time.time() - start_time
        piper.MotionCtrl_2(
            target_mode, 0x01, 100, is_mit_mode=0xAD if is_mit else 0x00
        )  # noqa: E501

        if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:  # noqa: E501
            return

        if elapsed_time > timeout:
            break

        time.sleep(1)

    raise TimeoutError


def get_disable_flag(piper: C_PiperInterface):
    msg = piper.GetArmLowSpdInfoMsgs()

    enable_flag = (
        not msg.motor_1.foc_status.driver_enable_status
        and not msg.motor_2.foc_status.driver_enable_status
        and not msg.motor_3.foc_status.driver_enable_status
        and not msg.motor_4.foc_status.driver_enable_status
        and not msg.motor_5.foc_status.driver_enable_status
        and not msg.motor_6.foc_status.driver_enable_status
    )
    return enable_flag


def disable_arm_ctrl(piper: C_PiperInterface, timeout: float = 5):
    timeout = 5
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time

        piper.DisableArm(7)
        piper.GripperCtrl(0, 1000, 0x02, 0)

        flag = get_disable_flag(piper)

        if flag:
            return

        if elapsed_time > timeout:
            break

        time.sleep(1)

    raise TimeoutError


def reset_piper_ctrl_mode(
    piper: C_PiperInterface, target_mode: int, max_retry: int = 5
) -> bool:
    if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:  # noqa: E501
        return True

    for _ in range(5):
        disable_arm_ctrl(piper, timeout=5)
        enable_arm_ctrl(piper, timeout=5)

        piper.MotionCtrl_2(target_mode, 0x01, 100, 0x00)

        if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:  # noqa: E501
            return True

    return False


def set_ctrl_method(
    piper: C_PiperInterface,
    is_mit: bool = False,
    mit_kp: float = 10.0,
    mit_kd: float = 0.8,
    mit_vel_ref: float = 45.0,
    mit_torque_ref: float = 0.0,
):
    if is_mit:
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
    else:
        piper.MotionCtrl_2(0x01, 0x01, 100, is_mit_mode=0x00)
