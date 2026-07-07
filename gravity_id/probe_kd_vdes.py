#!/usr/bin/env python3
"""Probe whether Piper MIT-mode kd / vel_ref behave like the MIT law.

Background: the firmware reinterprets the sent mit_kp (it holds position
with its own internal stiffness k_eff, see DEFLECTION_CALIBRATION.md), so
before building velocity feedforward on top of ``kd * (v_des - v)`` we
need to know whether the vel_ref field and kd gain are honored at all.

Three experiments on one follower arm (E1 -> E3 -> E2 order, most
informative first):

E1  static v_des injection. Hold a fixed pose, inject a constant
    vel_ref on one joint at a time. Ideal MIT law adds a constant torque
    kd*v_des, so the joint should settle displaced by kd*v_des/k_eff
    (k_eff from deflection_calibrations.json at the active kp).
    Conditions vary v_des at fixed kd and kd at fixed v_des, so the
    result discriminates: honored-per-law / scales-with-v-but-not-kd /
    ignored.

E3  sinusoid tracking on J3 at the deployment kd, with vel_ref = 0 vs
    vel_ref = analytic command derivative vs vel_ref = the arm's own
    measured velocity (the damping-cancellation variant). Reports
    amplitude ratio, phase lag and RMS error per condition.

E2  same sinusoid with vel_ref = 0 at low/high kd. If tracking is
    identical across kd, the sent kd is not acting as a damping gain
    (consistent with the kp finding).

The controller consumes vel_ref through its existing message-source
feedforward path, so this also verifies the ROS plumbing end to end:
the probe forces mit_velocity_feedforward=true,
mit_velocity_feedforward_source=message, deadband 0, and publishes
JointState commands with the velocity field filled.

Run INSIDE the holobrain container with the ROS environment sourced:

    docker exec -it holobrain bash
    source /opt/ros/humble/setup.bash
    python3 /opt/roboorchard/gravity_id/probe_kd_vdes.py --side left

PREREQUISITES:
  * single_ctrl controllers running; control mode Stop (no teleop /
    inference driving the arms). The probe stops the side's take_over
    muxer itself and respawns it on exit, then hard-refuses to run if
    any other publisher remains on /robot/<side>/joint_cmd.
  * E-stop within reach. The arm holds moderate poses; every injected
    torque is <= ~3.2 Nm and a 0.35 rad tracking-error guard aborts.

Gravity/friction compensation is disabled during the probe and every
touched parameter is restored on exit (including Ctrl-C).
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

from calibrate_kp import _respawn_muxer, _stop_muxer

HERE = Path(__file__).resolve().parent
CALIBRATION_STORE = HERE / "deflection_calibrations.json"

# Compact, tip-high pose from the validated measure_deflection grid.
TEST_POSE = [0.0, 0.6, -1.4, 0.0, 0.0, 0.0]

# E1: (kd, v_des) conditions per joint. Max injected torque 1.6*2 = 3.2 Nm.
E1_CONDITIONS = [
    (0.8, 1.0),
    (0.8, -1.0),
    (0.8, 2.0),
    (0.8, -2.0),
    (0.4, 2.0),
    (1.6, 2.0),
]
E1_JOINTS = (2, 3, 4)  # joints with fitted k_eff in the calibration store

# E2/E3 sinusoid on J3 around the test pose. Amplitude/frequency come
# from --amp/--freq; these bound what the CLI may request. J3 range on
# the validated pose grid is [-2.2, -0.5], so center -1.4 +/- 0.6 stays
# inside it; peak velocity is kept below the firmware's +/-45 rad/s
# vel_ref clamp with margin.
SINE_JOINT = 3
SINE_MAX_AMPLITUDE = 0.6
SINE_MAX_PEAK_VELOCITY = 6.0  # rad/s = amp * 2*pi*freq
SINE_DURATION = 12.0
SINE_SKIP = 2.5  # transient skipped before fitting

# kd=3.0 produced a violent >100 Hz torque limit cycle (audible buzz,
# +/-3..6 Nm effort chatter, joint stopped tracking) on J3 at kp=25 on
# 2026-07-06: the firmware's effective velocity-feedback gain is ~10x the
# sent kd (see E1), so kd=3 acts like kd~30. Stay well below; the chatter
# guard aborts the run if it starts.
E2_KDS = (0.1, 1.2)
# zero/deriv/measured inject vel_ref through the message source;
# estimator* exercise the PRODUCTION position_delta path (the controller
# finite-differences the command stream itself) at EMA alpha 0.2
# (deployment default, ~20 ms group delay at 200 Hz) and 0.5 (~5 ms).
E3_MODES = ("zero", "deriv", "estimator", "estimator_fast", "measured")

FORCED_PARAMS = {
    "mit_gravity_compensation_enabled": False,
    "mit_friction_compensation_enabled": False,
    "mit_velocity_feedforward": True,
    "mit_velocity_feedforward_source": "message",
    "mit_velocity_feedforward_deadband": 0.0,
}
SAVED_PARAM_NAMES = list(FORCED_PARAMS) + [
    "mit_kp", "mit_kd", "mit_velocity_feedforward_alpha",
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--exp", choices=("all", "e1", "e2", "e3"), default="all"
    )
    parser.add_argument(
        "--kd", type=float, default=0.8,
        help="Deployment kd used for E1 baseline conditions and E3.",
    )
    parser.add_argument(
        "--amp", type=float, default=0.3,
        help="Sinusoid amplitude on J3 (rad) for E2/E3, "
        f"max {SINE_MAX_AMPLITUDE}.",
    )
    parser.add_argument(
        "--freq", type=float, default=1.0,
        help="Sinusoid frequency (Hz) for E2/E3; amp*2*pi*freq must stay "
        f"below {SINE_MAX_PEAK_VELOCITY} rad/s.",
    )
    # 200 Hz matches the real teleop command rate (master controller
    # timer), which matters for the estimator conditions.
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--max-speed", type=float, default=0.15)
    parser.add_argument("--max-error", type=float, default=0.35)
    parser.add_argument(
        "--out-dir", default="/data/holobrain/kd_probe",
        help="Directory for CSV output (container-visible).",
    )
    parser.add_argument("--yes", action="store_true")
    return parser.parse_args()


def _load_k_eff(side: str, kp: float) -> list[float] | None:
    try:
        store = json.loads(CALIBRATION_STORE.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    entry = store.get(side, {}).get(f"{kp:g}")
    if not entry:
        return None
    ks = entry.get("offset_stiffness")
    return [float(k) for k in ks] if ks else None


def _solve3(a, b):
    """Solve a 3x3 linear system by Gaussian elimination with pivoting."""
    m = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    return [m[r][3] / m[r][r] for r in range(3)]


def _sine_fit(ts, ys, omega):
    """Least-squares fit y ~ a*sin(wt) + b*cos(wt) + c.

    Returns (amplitude, phase, offset, rms_residual); phase such that
    y ~ amp * sin(wt + phase) + c.
    """
    n = len(ts)
    if n < 8:
        return None
    s = [math.sin(omega * t) for t in ts]
    c = [math.cos(omega * t) for t in ts]
    ata = [
        [sum(x * x for x in s), sum(x * y for x, y in zip(s, c)), sum(s)],
        [sum(x * y for x, y in zip(s, c)), sum(x * x for x in c), sum(c)],
        [sum(s), sum(c), float(n)],
    ]
    atb = [
        sum(x * y for x, y in zip(s, ys)),
        sum(x * y for x, y in zip(c, ys)),
        sum(ys),
    ]
    sol = _solve3(ata, atb)
    if sol is None:
        return None
    a, b, off = sol
    resid = [
        y - (a * si + b * ci + off) for y, si, ci in zip(ys, s, c)
    ]
    rms = math.sqrt(sum(r * r for r in resid) / n)
    return math.hypot(a, b), math.atan2(b, a), off, rms


class _Prober(Node):
    def __init__(self, side: str) -> None:
        super().__init__(f"kd_vdes_prober_{side}")
        self.side = side
        self.cmd_topic = f"/robot/{side}/joint_cmd"
        self.state_topic = f"/puppet/joint_{side}"
        self.ctrl_node = f"/robot/{side}/robot_{side}_controller"
        self.latest: list[float] | None = None
        self.latest_velocity: list[float] | None = None
        self.latest_effort: list[float] | None = None
        self.sampling = False
        self.samples: list[list[float]] = []
        self.effort_samples: list[list[float]] = []
        self.recording = False
        self.state_series: list[tuple] = []
        self._eff_prev: list[float] | None = None
        self._eff_steps: list[tuple[float, float]] = []
        self._pub = None
        self.create_subscription(
            JointState, self.state_topic, self._on_state, 50
        )
        self._set_cli = self.create_client(
            SetParameters, f"{self.ctrl_node}/set_parameters"
        )
        self._get_cli = self.create_client(
            GetParameters, f"{self.ctrl_node}/get_parameters"
        )

    # -- state -------------------------------------------------------------
    def _on_state(self, msg: JointState) -> None:
        if len(msg.position) < 6:
            return
        self.latest = [float(p) for p in msg.position[:6]]
        if len(msg.velocity) >= 6:
            self.latest_velocity = [float(v) for v in msg.velocity[:6]]
        if len(msg.effort) >= 6:
            effort = [float(e) for e in msg.effort[:6]]
            if self._eff_prev is not None:
                step = max(
                    abs(a - b) for a, b in zip(effort, self._eff_prev)
                )
                now = time.monotonic()
                self._eff_steps.append((now, step))
                while self._eff_steps and self._eff_steps[0][0] < now - 1.0:
                    self._eff_steps.pop(0)
            self._eff_prev = effort
            self.latest_effort = effort
        if self.sampling:
            self.samples.append(list(self.latest))
            if self.latest_effort is not None:
                self.effort_samples.append(list(self.latest_effort))
        if self.recording:
            self.state_series.append(
                (
                    time.monotonic(),
                    list(self.latest),
                    list(self.latest_velocity or [0.0] * 6),
                    list(self.latest_effort or [0.0] * 6),
                )
            )

    def wait_for_state(self, timeout: float = 5.0) -> list[float]:
        deadline = time.monotonic() + timeout
        while self.latest is None:
            if time.monotonic() > deadline:
                raise SystemExit(
                    f"no JointState on {self.state_topic} within {timeout}s"
                )
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.latest)

    # -- publisher safety ----------------------------------------------------
    def assert_sole_publisher(self) -> None:
        count = self.count_publishers(self.cmd_topic)
        if count > 0:
            raise SystemExit(
                f"{count} publisher(s) still on {self.cmd_topic}. Stop "
                "aloha_orchestrator/inference app first (duplicate "
                "joint_cmd publishers caused the kd-buzz incident)."
            )
        self._pub = self.create_publisher(JointState, self.cmd_topic, 10)

    def publish_cmd(
        self, q6: list[float], v6: list[float] | None = None
    ) -> None:
        msg = JointState()
        msg.position = [float(v) for v in q6] + [0.0]
        msg.velocity = [
            float(v) for v in (v6 if v6 is not None else [0.0] * 6)
        ] + [0.0]
        self._pub.publish(msg)

    # -- parameters ----------------------------------------------------------
    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise SystemExit(f"service {client.srv_name} unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise SystemExit(f"call to {client.srv_name} failed")
        return future.result()

    def get_params(self, names: list[str]) -> dict[str, object]:
        req = GetParameters.Request(names=names)
        res = self._call(self._get_cli, req)
        out = {}
        for name, value in zip(names, res.values):
            if value.type == ParameterType.PARAMETER_BOOL:
                out[name] = value.bool_value
            elif value.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = value.double_value
            elif value.type == ParameterType.PARAMETER_STRING:
                out[name] = value.string_value
            else:
                out[name] = None
        return out

    def set_params(self, values: dict[str, object]) -> None:
        params = []
        for name, val in values.items():
            pv = ParameterValue()
            if isinstance(val, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = val
            elif isinstance(val, str):
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = val
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(val)
            params.append(ParameterMsg(name=name, value=pv))
        res = self._call(
            self._set_cli, SetParameters.Request(parameters=params)
        )
        for name, result in zip(values, res.results):
            if not result.successful:
                raise SystemExit(
                    f"set_parameters rejected {name}: {result.reason}"
                )

    # -- motion ----------------------------------------------------------------
    def _spin_sleep(self, duration: float) -> None:
        end = time.monotonic() + duration
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.02))

    def _check_error(self, target: list[float], max_error: float) -> None:
        if self.latest is None:
            return
        for j, (cmd, meas) in enumerate(zip(target, self.latest)):
            if abs(cmd - meas) > max_error:
                raise RuntimeError(
                    f"joint{j + 1} error {abs(cmd - meas):.3f} rad exceeds "
                    f"--max-error {max_error}; aborting for safety"
                )
        # Torque chatter guard: the kd=3 buzz showed as sustained 3..6 Nm
        # effort jumps at every 200 Hz feedback sample. Healthy runs stay
        # below ~0.5 Nm per step.
        big = sum(1 for _, s in self._eff_steps if s > 1.5)
        if big >= 30:
            raise RuntimeError(
                f"torque chatter: {big} effort steps >1.5 Nm in the last "
                "second; aborting (lower kd)"
            )

    def ramp_to(
        self, target: list[float], rate: float, max_speed: float
    ) -> None:
        start = self.wait_for_state()
        dist = max(abs(t - s) for t, s in zip(target, start))
        steps = max(2, int(math.ceil(dist / max_speed * rate)))
        for i in range(1, steps + 1):
            a = i / steps
            q = [s + a * (t - s) for s, t in zip(start, target)]
            self.publish_cmd(q)
            self._spin_sleep(1.0 / rate)

    def hold_and_sample(
        self,
        target: list[float],
        vel: list[float],
        rate: float,
        settle: float,
        seconds: float,
        max_error: float,
    ) -> tuple[list[float], list[float], list[float], int]:
        """Hold target with vel injected; returns (mean, std, eff, n)."""
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            self.publish_cmd(target, vel)
            self._check_error(target, max_error)
            self._spin_sleep(1.0 / rate)
        self.samples, self.effort_samples = [], []
        self.sampling = True
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_cmd(target, vel)
            self._check_error(target, max_error)
            self._spin_sleep(1.0 / rate)
        self.sampling = False
        samples = self.samples
        if len(samples) < 10:
            raise SystemExit(
                f"only {len(samples)} samples on {self.state_topic}"
            )
        mean = [statistics.fmean(s[j] for s in samples) for j in range(6)]
        std = [
            statistics.pstdev([s[j] for s in samples]) for j in range(6)
        ]
        eff = [0.0] * 6
        if self.effort_samples:
            eff = [
                statistics.fmean(s[j] for s in self.effort_samples)
                for j in range(6)
            ]
        return mean, std, eff, len(samples)

    def run_sinusoid(
        self,
        center_pose: list[float],
        joint: int,
        amplitude: float,
        freq_hz: float,
        duration: float,
        vff_mode: str,
        rate: float,
        max_error: float,
    ) -> tuple[list[tuple], list[tuple]]:
        """Drive one joint sinusoidally; returns (cmd_series, state_series).

        vff_mode: 'zero' | 'deriv' | 'measured' selects the velocity sent
        in the command message for the driven joint.
        """
        jj = joint - 1
        omega = 2.0 * math.pi * freq_hz
        cmd_series: list[tuple] = []
        self.state_series = []
        self.recording = True
        t0 = time.monotonic()
        try:
            while True:
                t = time.monotonic() - t0
                if t > duration:
                    break
                q = list(center_pose)
                q[jj] += amplitude * math.sin(omega * t)
                vel = [0.0] * 6
                if vff_mode == "deriv":
                    vel[jj] = amplitude * omega * math.cos(omega * t)
                elif vff_mode == "measured":
                    if self.latest_velocity is not None:
                        vel[jj] = self.latest_velocity[jj]
                self.publish_cmd(q, vel)
                cmd_series.append((time.monotonic(), q[jj], vel[jj]))
                self._check_error(q, max_error)
                self._spin_sleep(1.0 / rate)
        finally:
            self.recording = False
        return cmd_series, self.state_series


def _run_e1(
    node: _Prober,
    args: argparse.Namespace,
    k_eff: list[float] | None,
    out_dir: str,
    stamp: str,
) -> None:
    print("\n=== E1: static v_des injection ===")
    rows = []
    for joint in E1_JOINTS:
        jj = joint - 1
        print(f"-- joint {joint} --")
        node.set_params({"mit_kd": args.kd})
        base_mean, _, base_eff, _ = node.hold_and_sample(
            TEST_POSE, [0.0] * 6, args.rate, args.settle, args.seconds,
            args.max_error,
        )
        for kd, v_des in E1_CONDITIONS:
            node.set_params({"mit_kd": kd})
            vel = [0.0] * 6
            vel[jj] = v_des
            cond_mean, cond_std, cond_eff, n = node.hold_and_sample(
                TEST_POSE, vel, args.rate, args.settle, args.seconds,
                args.max_error,
            )
            node.set_params({"mit_kd": args.kd})
            next_mean, _, next_eff, _ = node.hold_and_sample(
                TEST_POSE, [0.0] * 6, args.rate, args.settle,
                args.seconds, args.max_error,
            )
            base_avg = [
                (b + a) / 2.0 for b, a in zip(base_mean, next_mean)
            ]
            eff_avg = [(b + a) / 2.0 for b, a in zip(base_eff, next_eff)]
            dq = cond_mean[jj] - base_avg[jj]
            deff = cond_eff[jj] - eff_avg[jj]
            tau = kd * v_des
            pred = None
            if k_eff and jj < len(k_eff) and k_eff[jj] > 0:
                pred = tau / k_eff[jj]
            pred_s = f"{pred:+.4f}" if pred is not None else "   n/a"
            ratio_s = (
                f"{dq / pred:5.2f}" if pred not in (None, 0.0) else "  n/a"
            )
            print(
                f"  kd={kd:<4g} v={v_des:+.1f}  tau={tau:+.2f}Nm  "
                f"dq={dq:+.4f} rad (pred {pred_s}, x{ratio_s})  "
                f"deff={deff:+.3f}  std={cond_std[jj]:.4f} n={n}"
            )
            row = {
                "side": args.side, "joint": joint, "kd": kd,
                "v_des": v_des, "tau_inj": tau, "n_samples": n,
                "dq": round(dq, 6), "deff": round(deff, 6),
                "pred_dq": round(pred, 6) if pred is not None else "",
            }
            for j in range(6):
                row[f"dq{j + 1}"] = round(
                    cond_mean[j] - base_avg[j], 6
                )
                row[f"std{j + 1}"] = round(cond_std[j], 6)
            rows.append(row)
            base_mean, base_eff = next_mean, next_eff
    path = os.path.join(out_dir, f"e1_static_{args.side}_{stamp}.csv")
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"E1 rows -> {path}")


def _analyze_sinusoid(
    label: str,
    cmd_series: list[tuple],
    state_series: list[tuple],
    joint: int,
    freq_hz: float,
) -> dict | None:
    jj = joint - 1
    omega = 2.0 * math.pi * freq_hz
    if not cmd_series or not state_series:
        print(f"  {label}: no data")
        return None
    t0 = cmd_series[0][0]
    cmd_pts = [
        (t - t0, q) for t, q, _ in cmd_series if t - t0 >= SINE_SKIP
    ]
    meas_pts = [
        (t - t0, qs[jj])
        for t, qs, _, _ in state_series
        if t - t0 >= SINE_SKIP
    ]
    cmd_fit = _sine_fit(
        [p[0] for p in cmd_pts], [p[1] for p in cmd_pts], omega
    )
    meas_fit = _sine_fit(
        [p[0] for p in meas_pts], [p[1] for p in meas_pts], omega
    )
    if cmd_fit is None or meas_fit is None:
        print(f"  {label}: fit failed")
        return None
    amp_ratio = meas_fit[0] / cmd_fit[0] if cmd_fit[0] else float("nan")
    dphi = meas_fit[1] - cmd_fit[1]
    while dphi > math.pi:
        dphi -= 2.0 * math.pi
    while dphi < -math.pi:
        dphi += 2.0 * math.pi
    lag_ms = -dphi / omega * 1000.0
    # RMS tracking error from nearest-neighbour pairing (both series are
    # dense at ~50 Hz, so zero-order interpolation is adequate).
    errs = []
    ci = 0
    for tm, qm in meas_pts:
        while (
            ci + 1 < len(cmd_pts) and cmd_pts[ci + 1][0] <= tm
        ):
            ci += 1
        errs.append(qm - cmd_pts[ci][1])
    rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else 0.0
    print(
        f"  {label:<28} amp_ratio={amp_ratio:.3f}  "
        f"phase_lag={math.degrees(-dphi):+.1f} deg ({lag_ms:+.0f} ms)  "
        f"rms_err={rms * 1000:.1f} mrad"
    )
    return {
        "label": label,
        "amp_ratio": round(amp_ratio, 4),
        "phase_lag_deg": round(math.degrees(-dphi), 2),
        "lag_ms": round(lag_ms, 1),
        "rms_err_mrad": round(rms * 1000.0, 2),
    }


def _dump_series(
    path: str,
    cmd_series: list[tuple],
    state_series: list[tuple],
    joint: int,
) -> None:
    jj = joint - 1
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["kind", "t", "q", "v", "eff"])
        for t, q, v in cmd_series:
            writer.writerow(["cmd", f"{t:.4f}", f"{q:.6f}", f"{v:.4f}", ""])
        for t, qs, vs, effs in state_series:
            writer.writerow(
                [
                    "meas", f"{t:.4f}", f"{qs[jj]:.6f}",
                    f"{vs[jj]:.4f}", f"{effs[jj]:.4f}",
                ]
            )


def _run_sine_condition(
    node: _Prober,
    args: argparse.Namespace,
    label: str,
    kd: float,
    vff_mode: str,
    out_dir: str,
    stamp: str,
    summaries: list,
) -> None:
    params: dict[str, object] = {"mit_kd": kd}
    if vff_mode.startswith("estimator"):
        params["mit_velocity_feedforward_source"] = "position_delta"
        params["mit_velocity_feedforward_alpha"] = (
            0.5 if vff_mode == "estimator_fast" else 0.2
        )
    else:
        params["mit_velocity_feedforward_source"] = "message"
    node.set_params(params)
    node.ramp_to(TEST_POSE, args.rate, args.max_speed)
    node.hold_and_sample(
        TEST_POSE, [0.0] * 6, args.rate, 1.5, 0.5, args.max_error
    )
    try:
        cmd_series, state_series = node.run_sinusoid(
            TEST_POSE, SINE_JOINT, args.amp, args.freq,
            SINE_DURATION, vff_mode, args.rate, args.max_error,
        )
    finally:
        # Never leave an experimental kd active for the following ramp
        # (the 2026-07-06 buzz continued through the return-home ramp
        # because kd=3 was only restored after it).
        node.set_params({"mit_kd": args.kd})
    result = _analyze_sinusoid(
        label, cmd_series, state_series, SINE_JOINT, args.freq
    )
    if result is not None:
        result["kd"] = kd
        result["vff_mode"] = vff_mode
        summaries.append(result)
    safe = label.replace(" ", "_").replace("=", "")
    _dump_series(
        os.path.join(out_dir, f"sine_{safe}_{args.side}_{stamp}.csv"),
        cmd_series, state_series, SINE_JOINT,
    )


def main() -> None:
    args = _parse_args()
    peak_vel = args.amp * 2.0 * math.pi * args.freq
    if not 0.0 < args.amp <= SINE_MAX_AMPLITUDE:
        raise SystemExit(
            f"--amp {args.amp:g} outside (0, {SINE_MAX_AMPLITUDE}]; the "
            "J3 excursion must stay inside the validated pose range."
        )
    if args.freq <= 0.0 or peak_vel > SINE_MAX_PEAK_VELOCITY:
        raise SystemExit(
            f"--freq {args.freq:g} gives peak velocity {peak_vel:.2f} "
            f"rad/s > {SINE_MAX_PEAK_VELOCITY} rad/s; lower --amp or "
            "--freq."
        )
    print(
        f"side={args.side} exp={args.exp} deploy_kd={args.kd:g} "
        f"sine amp={args.amp:g} rad freq={args.freq:g} Hz "
        f"(peak vel {peak_vel:.2f} rad/s)"
    )
    if not args.yes:
        reply = input(
            "The follower arm will hold/move around pose "
            f"{TEST_POSE} with gravity comp OFF, injected torques up to "
            f"3.2 Nm and a +/-{args.amp:g} rad {args.freq:g} Hz sinusoid "
            "on J3. The side's take_over muxer is stopped for the run. "
            "Control mode Stop? E-stop in reach? [y/N] "
        )
        if reply.strip().lower() != "y":
            raise SystemExit("aborted")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)

    muxer_argv = _stop_muxer(args.side)

    rclpy.init()
    node = _Prober(args.side)
    exit_code = 0
    home: list[float] | None = None
    saved: dict[str, object] = {}
    try:
        node.assert_sole_publisher()
        home = node.wait_for_state()
        print(f"start pose: {[round(v, 3) for v in home]}")

        saved = node.get_params(SAVED_PARAM_NAMES)
        print(f"saved controller params: {saved}")
        kp = saved.get("mit_kp")
        k_eff = _load_k_eff(args.side, kp) if kp else None
        if k_eff:
            print(f"k_eff at kp={kp:g}: {k_eff}")
        else:
            print(f"WARNING: no k_eff calibration at kp={kp}; E1 "
                  "predictions unavailable.")

        forced = dict(FORCED_PARAMS)
        forced["mit_kd"] = args.kd
        node.set_params(forced)

        node.ramp_to(TEST_POSE, args.rate, args.max_speed)

        summaries: list[dict] = []
        if args.exp in ("all", "e1"):
            _run_e1(node, args, k_eff, out_dir, stamp)
        if args.exp in ("all", "e3"):
            print("\n=== E3: sinusoid, vel feedforward modes "
                  f"(kd={args.kd:g}) ===")
            for mode in E3_MODES:
                _run_sine_condition(
                    node, args, f"e3 kd={args.kd:g} vff={mode}",
                    args.kd, mode, out_dir, stamp, summaries,
                )
        if args.exp in ("all", "e2"):
            print("\n=== E2: sinusoid, kd sweep (vel_ref=0) ===")
            for kd in E2_KDS:
                _run_sine_condition(
                    node, args, f"e2 kd={kd:g} vff=zero",
                    kd, "zero", out_dir, stamp, summaries,
                )
        if summaries:
            path = os.path.join(
                out_dir, f"sine_summary_{args.side}_{stamp}.csv"
            )
            with open(path, "w", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(summaries[0]))
                writer.writeheader()
                writer.writerows(summaries)
            print(f"\nsine summaries -> {path}")
    except (RuntimeError, KeyboardInterrupt, SystemExit) as exc:
        exit_code = 1
        print(f"\nstopping: {exc}")
    finally:
        # Restore params BEFORE moving: the return ramp must run at the
        # deployment kd/comp settings, not whatever the experiment forced.
        if saved:
            try:
                print(f"restoring controller params: {saved}")
                order = sorted(
                    saved, key=lambda k: 0 if k.startswith("mit_k") else 1
                )
                node.set_params(
                    {k: saved[k] for k in order if saved[k] is not None}
                )
            except Exception as exc:
                print(f"WARNING: param restore failed: {exc}")
        try:
            if home is not None and node._pub is not None:
                print("returning to start pose...")
                node.ramp_to(home, args.rate, args.max_speed)
        except Exception as exc:
            print(f"WARNING: return ramp failed: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        if muxer_argv:
            _respawn_muxer(muxer_argv)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
