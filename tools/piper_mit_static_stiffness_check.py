#!/usr/bin/env python3
"""Run an automated Piper MIT static stiffness check for one joint.

This script intentionally bypasses the RoboOrchard ROS controller. Run it only
when no other process is commanding the same Piper CAN interface.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import math
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


RAD_PER_MDEG = math.pi / 180000.0
DEFAULT_OUT_DIR = "/data/holobrain/mit_static_stiffness"
MIT_EXECUTION_SCALE = 4.0
MIT_P_RANGE = (-12.5, 12.5, 16)
MIT_V_RANGE = (-45.0, 45.0, 12)
MIT_KP_RANGE = (0.0, 500.0, 12)
MIT_KD_RANGE = (-5.0, 5.0, 12)
MIT_T_RANGE = (-8.0, 8.0, 8)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stream true Piper JointMitCtrl frames and log displacement/effort "
            "pairs for static stiffness checks."
        )
    )
    parser.add_argument("--can", default="can_left", help="CAN device name.")
    parser.add_argument(
        "--joint",
        type=int,
        default=6,
        choices=range(1, 7),
        metavar="{1..6}",
        help="Joint to analyze.",
    )
    parser.add_argument(
        "--kp", type=float, default=1.0, help="Target joint MIT kp."
    )
    parser.add_argument(
        "--kd", type=float, default=0.0, help="Target joint MIT kd."
    )
    parser.add_argument(
        "--t-ref",
        type=float,
        default=0.0,
        help="Target joint MIT feedforward torque reference.",
    )
    parser.add_argument(
        "--other-kp",
        type=float,
        default=None,
        help=(
            "MIT kp for non-target joints held at their start pose. "
            "Defaults to --kp."
        ),
    )
    parser.add_argument(
        "--other-kd",
        type=float,
        default=0.0,
        help="MIT kd for non-target joints.",
    )
    parser.add_argument(
        "--other-t-ref",
        type=float,
        default=0.0,
        help="MIT feedforward torque for non-target joints.",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=200.0,
        help="Command/logging loop rate. Use at least 200 Hz when possible.",
    )
    parser.add_argument(
        "--auto-offset-rad",
        type=float,
        default=0.02,
        help="Reference offset e in radians.",
    )
    parser.add_argument(
        "--auto-mode",
        choices=("position", "torque"),
        default="position",
        help=(
            "Automated plus/minus stimulus. 'position' commands q0+e/q0-e; "
            "'torque' holds q0 and commands tau_ff +/- --auto-t-ref-nm."
        ),
    )
    parser.add_argument(
        "--auto-t-ref-nm",
        type=float,
        default=0.05,
        help="Feedforward torque magnitude for --auto-mode torque.",
    )
    parser.add_argument(
        "--auto-settle-s",
        type=float,
        default=0.5,
        help="Settling time before averaging effort.",
    )
    parser.add_argument(
        "--auto-hold-s",
        type=float,
        default=1.0,
        help="Total hold time for each q0+e or q0-e phase.",
    )
    parser.add_argument(
        "--auto-cycles",
        type=int,
        default=1,
        help="Number of plus/minus cycles.",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help="Directory for CSV logs.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Exact CSV path. Overrides --out-dir naming.",
    )
    parser.add_argument(
        "--print-every-s",
        type=float,
        default=0.5,
        help="Live status print interval. 0 disables prints.",
    )
    parser.add_argument(
        "--enable-timeout-s",
        type=float,
        default=5.0,
        help="Timeout while waiting for EnablePiper().",
    )
    parser.add_argument(
        "--no-enable",
        action="store_true",
        help="Do not call EnablePiper before entering MIT mode.",
    )
    parser.add_argument(
        "--mode-refresh-s",
        type=float,
        default=1.0,
        help="Refresh MotionCtrl_2 MIT mode at this period. 0 disables refresh.",
    )
    parser.add_argument(
        "--neutralize-s",
        type=float,
        default=0.5,
        help="Seconds to stream kp=kd=t_ref=0 on exit.",
    )
    parser.add_argument(
        "--allow-controller-conflict",
        action="store_true",
        help=(
            "Run even if a RoboOrchard single_ctrl process appears to own "
            "the selected CAN device. By default this is refused because "
            "the ROS controller can overwrite direct MIT frames."
        ),
    )
    return parser.parse_args()


def _joint_positions_rad(piper: C_PiperInterface_V2) -> list[float]:
    msg = piper.GetArmJointMsgs().joint_state
    raw = [
        msg.joint_1,
        msg.joint_2,
        msg.joint_3,
        msg.joint_4,
        msg.joint_5,
        msg.joint_6,
    ]
    return [value * RAD_PER_MDEG for value in raw]


def _high_speed(piper: C_PiperInterface_V2) -> tuple[list[float], list[int]]:
    msg = piper.GetArmHighSpdInfoMsgs()
    motors = [
        msg.motor_1,
        msg.motor_2,
        msg.motor_3,
        msg.motor_4,
        msg.motor_5,
        msg.motor_6,
    ]
    # Existing RoboOrchard code treats motor_speed / 1000 as rad/s.
    velocities = [motor.motor_speed / 1000.0 for motor in motors]
    efforts = [int(motor.effort) for motor in motors]
    return velocities, efforts


def _effort_nm_div1000(raw_effort: int) -> float:
    return raw_effort / 1000.0


def _effort_nm_fw18(joint: int, raw_effort: int) -> float:
    scale = 4.0 if joint <= 3 else 1.0
    return raw_effort * scale / 1000.0


def _csv_fields() -> list[str]:
    fields = [
        "sample_idx",
        "t_s",
        "wall_time_iso",
        "can",
        "target_joint",
        "rate_hz",
        "target_kp",
        "target_kd",
        "target_t_ref",
        "other_kp",
        "other_kd",
        "other_t_ref",
        "phase",
        "phase_sign",
        "phase_t_s",
        "q0_rad",
        "command_offset_rad",
        "command_t_ref_offset_nm",
        "included_in_auto_mean",
        "p_des_rad",
        "q_rad",
        "delta_rad",
        "delta_cmd_rad",
        "vel_rad_s",
        "vel_error_rad_s",
        "raw_effort",
        "effort_nm_div1000",
        "effort_nm_fw18_j123",
        "expected_tau_kp_nm",
        "expected_tau_4kp_nm",
        "mit_requested_input_tau_nm",
        "mit_input_tau_nm",
        "mit_executed_4x_tau_nm",
        "encoded_t_ref_nm",
    ]
    for prefix in ("q", "p_des", "delta"):
        fields.extend(f"{prefix}{idx}_rad" for idx in range(1, 7))
    fields.extend(f"vel{idx}_rad_s" for idx in range(1, 7))
    fields.extend(f"raw_effort{idx}" for idx in range(1, 7))
    fields.extend(f"effort{idx}_nm_div1000" for idx in range(1, 7))
    fields.extend(f"effort{idx}_nm_fw18_j123" for idx in range(1, 7))
    fields.extend(f"kp{idx}" for idx in range(1, 7))
    fields.extend(f"kd{idx}" for idx in range(1, 7))
    fields.extend(f"t_ref{idx}" for idx in range(1, 7))
    return fields


def _make_output_path(args: argparse.Namespace) -> Path:
    if args.output:
        return Path(args.output)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{args.can}_j{args.joint}_kp{args.kp:g}_{stamp}.csv"
    return Path(args.out_dir) / filename


@dataclasses.dataclass
class AutoPhaseStats:
    phase: str
    sign: int
    samples: int = 0
    raw_sum: float = 0.0
    effort_div1000_sum: float = 0.0
    effort_fw18_sum: float = 0.0
    mit_input_tau_sum: float = 0.0

    def add(self, joint: int, raw_effort: int, mit_input_tau_nm: float) -> None:
        self.samples += 1
        self.raw_sum += raw_effort
        self.effort_div1000_sum += _effort_nm_div1000(raw_effort)
        self.effort_fw18_sum += _effort_nm_fw18(joint, raw_effort)
        self.mit_input_tau_sum += mit_input_tau_nm

    @property
    def raw_mean(self) -> float:
        return self.raw_sum / self.samples

    @property
    def effort_div1000_mean(self) -> float:
        return self.effort_div1000_sum / self.samples

    @property
    def effort_fw18_mean(self) -> float:
        return self.effort_fw18_sum / self.samples

    @property
    def mit_input_tau_mean(self) -> float:
        return self.mit_input_tau_sum / self.samples


def _process_params_files(command: str) -> list[Path]:
    try:
        parts = shlex.split(command)
    except ValueError:
        parts = command.split()
    params_files = []
    for idx, part in enumerate(parts[:-1]):
        if part == "--params-file":
            params_files.append(Path(parts[idx + 1]))
    return params_files


def _params_file_mentions_can(params_file: Path, can_name: str) -> bool:
    try:
        params_text = params_file.read_text()
    except OSError:
        return False
    pattern = re.compile(rf"^\s*can_port:\s*{re.escape(can_name)}\s*$")
    return any(pattern.match(line) for line in params_text.splitlines())


def _active_controller_conflicts(can_name: str) -> list[str]:
    result = subprocess.run(
        ["ps", "-eo", "pid,args"],
        check=True,
        capture_output=True,
        text=True,
    )
    conflicts = []
    possible_conflicts = []
    for line in result.stdout.splitlines():
        if "single_ctrl" not in line:
            continue
        stripped_line = line.strip()
        params_files = _process_params_files(stripped_line)
        params_match = any(
            _params_file_mentions_can(params_file, can_name)
            for params_file in params_files
        )
        command_match = (
            f"can_port:={can_name}" in stripped_line
            or f"can_port={can_name}" in stripped_line
            or (can_name in stripped_line and "robo_orchard_piper_ros2" in stripped_line)
        )
        if params_match or command_match:
            conflicts.append(stripped_line)
        elif params_files:
            possible_conflicts.append(stripped_line)
    if conflicts:
        return conflicts
    return [
        "possible controller conflict, could not match can_port: " + line
        for line in possible_conflicts
    ]


def _check_no_controller_conflict(args: argparse.Namespace) -> None:
    conflicts = _active_controller_conflicts(args.can)
    if not conflicts:
        return
    message = (
        f"Found {len(conflicts)} active RoboOrchard controller process(es) "
        f"that may command {args.can}. Stop them before running this direct "
        "CAN MIT test, or pass --allow-controller-conflict if you are only "
        "debugging logging. Conflicts:\n  "
        + "\n  ".join(conflicts)
    )
    if args.allow_controller_conflict:
        print("WARNING: " + message, file=sys.stderr)
        return
    raise RuntimeError(message)


def _sdk_float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    span = value_max - value_min
    return int((value - value_min) * float((1 << bits) - 1) / span)


def _sdk_uint_to_float(code: int, value_min: float, value_max: float, bits: int) -> float:
    span = value_max - value_min
    return code * span / float((1 << bits) - 1) + value_min


def _sdk_encoded_float(value: float, value_range: tuple[float, float, int]) -> float:
    value_min, value_max, bits = value_range
    code = _sdk_float_to_uint(value, value_min, value_max, bits)
    return _sdk_uint_to_float(code, value_min, value_max, bits)


def _print_command_scale(args: argparse.Namespace) -> None:
    if args.auto_mode == "torque":
        requested_plus = args.t_ref + args.auto_t_ref_nm
        requested_minus = args.t_ref - args.auto_t_ref_nm
        encoded_plus = _sdk_encoded_float(requested_plus, MIT_T_RANGE)
        encoded_minus = _sdk_encoded_float(requested_minus, MIT_T_RANGE)
        requested_delta = requested_plus - requested_minus
        encoded_delta = encoded_plus - encoded_minus
        print(
            "feedforward torque step: "
            f"requested plus/minus={requested_plus:+.6f}/"
            f"{requested_minus:+.6f} Nm, "
            f"encoded plus/minus={encoded_plus:+.6f}/"
            f"{encoded_minus:+.6f} Nm, "
            f"encoded diff={encoded_delta:.6f} Nm; "
            f"4x execution hypothesis gives "
            f"{MIT_EXECUTION_SCALE * encoded_delta:.6f} Nm difference"
        )
        if abs(encoded_delta) < 1e-9:
            print(
                "WARNING: requested +/- torque quantizes to the same SDK "
                "t_ref code; increase --auto-t-ref-nm.",
                file=sys.stderr,
            )
        elif abs(encoded_delta - requested_delta) > 0.25 * max(abs(requested_delta), 1e-9):
            print(
                "WARNING: requested torque is close to the 8-bit t_ref "
                "resolution; alpha will use the encoded effective torque.",
                file=sys.stderr,
            )
        return

    one_side = args.kp * args.auto_offset_rad
    two_side = 2.0 * one_side
    print(
        "position-step torque from K*e: "
        f"one-sided requested input={one_side:.6f} Nm, "
        f"plus-minus requested input difference={two_side:.6f} Nm; "
        f"4x execution hypothesis gives "
        f"{MIT_EXECUTION_SCALE * two_side:.6f} Nm difference"
    )


def _enable_piper(piper: C_PiperInterface_V2, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            if piper.EnablePiper():
                return
        except Exception as exc:  # noqa: BLE001
            last_error = exc
        time.sleep(0.01)
    if last_error is not None:
        raise TimeoutError(f"EnablePiper failed: {last_error}")
    raise TimeoutError("EnablePiper timed out")


def _enter_mit_mode(piper: C_PiperInterface_V2) -> None:
    # SDK MIT joint-control demo uses ctrl_mode=0x01, move_mode=0x04,
    # speed=0, is_mit_mode=0xAD.
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)


def _send_mit_frame_set(
    piper: C_PiperInterface_V2,
    p_des: list[float],
    kp: list[float],
    kd: list[float],
    t_ref: list[float],
) -> None:
    for idx in range(6):
        piper.JointMitCtrl(idx + 1, p_des[idx], 0.0, kp[idx], kd[idx], t_ref[idx])


def _neutralize(
    piper: C_PiperInterface_V2,
    last_q: list[float],
    duration_s: float,
    rate_hz: float,
) -> None:
    if duration_s <= 0.0:
        return
    period_s = 1.0 / max(rate_hz, 1.0)
    deadline = time.monotonic() + duration_s
    zero = [0.0] * 6
    while time.monotonic() < deadline:
        try:
            _enter_mit_mode(piper)
            _send_mit_frame_set(piper, last_q, zero, zero, zero)
        except Exception as exc:  # noqa: BLE001
            print(f"neutralize warning: {exc}", file=sys.stderr)
            break
        time.sleep(period_s)


def _mit_requested_input_tau_nm(
    *,
    target_idx: int,
    p_des: list[float],
    q: list[float],
    vel: list[float],
    kp: list[float],
    kd: list[float],
    t_ref: list[float],
) -> float:
    return (
        kp[target_idx] * (p_des[target_idx] - q[target_idx])
        + kd[target_idx] * (0.0 - vel[target_idx])
        + t_ref[target_idx]
    )


def _mit_input_tau_nm(
    *,
    target_idx: int,
    p_des: list[float],
    q: list[float],
    vel: list[float],
    kp: list[float],
    kd: list[float],
    t_ref: list[float],
) -> float:
    p_des_eff = _sdk_encoded_float(p_des[target_idx], MIT_P_RANGE)
    vel_ref_eff = _sdk_encoded_float(0.0, MIT_V_RANGE)
    kp_eff = _sdk_encoded_float(kp[target_idx], MIT_KP_RANGE)
    kd_eff = _sdk_encoded_float(kd[target_idx], MIT_KD_RANGE)
    t_ref_eff = _sdk_encoded_float(t_ref[target_idx], MIT_T_RANGE)
    return kp_eff * (p_des_eff - q[target_idx]) + kd_eff * (
        vel_ref_eff - vel[target_idx]
    ) + t_ref_eff


def _row(
    *,
    args: argparse.Namespace,
    sample_idx: int,
    t_s: float,
    p_des: list[float],
    q: list[float],
    vel: list[float],
    raw_effort: list[int],
    kp: list[float],
    kd: list[float],
    t_ref: list[float],
    phase: str = "auto",
    phase_sign: int = 0,
    phase_t_s: float = 0.0,
    q0: list[float] | None = None,
    command_offset_rad: float = 0.0,
    command_t_ref_offset_nm: float = 0.0,
    included_in_auto_mean: bool = False,
) -> dict[str, Any]:
    target_idx = args.joint - 1
    delta = [q[idx] - p_des[idx] for idx in range(6)]
    target_delta = delta[target_idx]
    target_delta_cmd = -target_delta
    target_vel_error = -vel[target_idx]
    target_raw_effort = raw_effort[target_idx]
    mit_requested_input_tau_nm = _mit_requested_input_tau_nm(
        target_idx=target_idx,
        p_des=p_des,
        q=q,
        vel=vel,
        kp=kp,
        kd=kd,
        t_ref=t_ref,
    )
    mit_input_tau_nm = _mit_input_tau_nm(
        target_idx=target_idx,
        p_des=p_des,
        q=q,
        vel=vel,
        kp=kp,
        kd=kd,
        t_ref=t_ref,
    )
    encoded_t_ref_nm = _sdk_encoded_float(t_ref[target_idx], MIT_T_RANGE)
    q0_value = "" if q0 is None else q0[target_idx]
    row: dict[str, Any] = {
        "sample_idx": sample_idx,
        "t_s": f"{t_s:.6f}",
        "wall_time_iso": dt.datetime.now().isoformat(timespec="milliseconds"),
        "can": args.can,
        "target_joint": args.joint,
        "rate_hz": args.rate_hz,
        "target_kp": args.kp,
        "target_kd": args.kd,
        "target_t_ref": args.t_ref,
        "other_kp": args.other_kp,
        "other_kd": args.other_kd,
        "other_t_ref": args.other_t_ref,
        "phase": phase,
        "phase_sign": phase_sign,
        "phase_t_s": f"{phase_t_s:.6f}",
        "q0_rad": q0_value,
        "command_offset_rad": command_offset_rad,
        "command_t_ref_offset_nm": command_t_ref_offset_nm,
        "included_in_auto_mean": int(included_in_auto_mean),
        "p_des_rad": p_des[target_idx],
        "q_rad": q[target_idx],
        "delta_rad": target_delta,
        "delta_cmd_rad": target_delta_cmd,
        "vel_rad_s": vel[target_idx],
        "vel_error_rad_s": target_vel_error,
        "raw_effort": target_raw_effort,
        "effort_nm_div1000": _effort_nm_div1000(target_raw_effort),
        "effort_nm_fw18_j123": _effort_nm_fw18(args.joint, target_raw_effort),
        "expected_tau_kp_nm": args.kp * target_delta_cmd,
        "expected_tau_4kp_nm": MIT_EXECUTION_SCALE * args.kp * target_delta_cmd,
        "mit_requested_input_tau_nm": mit_requested_input_tau_nm,
        "mit_input_tau_nm": mit_input_tau_nm,
        "mit_executed_4x_tau_nm": MIT_EXECUTION_SCALE * mit_input_tau_nm,
        "encoded_t_ref_nm": encoded_t_ref_nm,
    }
    for idx in range(6):
        joint = idx + 1
        row[f"q{joint}_rad"] = q[idx]
        row[f"p_des{joint}_rad"] = p_des[idx]
        row[f"delta{joint}_rad"] = delta[idx]
        row[f"vel{joint}_rad_s"] = vel[idx]
        row[f"raw_effort{joint}"] = raw_effort[idx]
        row[f"effort{joint}_nm_div1000"] = _effort_nm_div1000(raw_effort[idx])
        row[f"effort{joint}_nm_fw18_j123"] = _effort_nm_fw18(joint, raw_effort[idx])
        row[f"kp{joint}"] = kp[idx]
        row[f"kd{joint}"] = kd[idx]
        row[f"t_ref{joint}"] = t_ref[idx]
    return row


def _print_row_status(row: dict[str, Any]) -> None:
    print(
        "t={t_s:.2f}s phase={phase} cmd_delta={cmd_delta:+.5f} rad "
        "vel_err={vel_err:+.4f} rad/s mit_input_encoded={mit_input:+.4f} Nm "
        "raw_effort={raw} effort/1000={eff:+.4f} Nm "
        "fw18={fw:+.4f} Nm".format(
            t_s=float(row["t_s"]),
            phase=row["phase"],
            cmd_delta=float(row["delta_cmd_rad"]),
            vel_err=float(row["vel_error_rad_s"]),
            mit_input=float(row["mit_input_tau_nm"]),
            raw=row["raw_effort"],
            eff=float(row["effort_nm_div1000"]),
            fw=float(row["effort_nm_fw18_j123"]),
        ),
        flush=True,
    )


def _print_auto_summary(
    *,
    args: argparse.Namespace,
    plus_stats: AutoPhaseStats,
    minus_stats: AutoPhaseStats,
) -> None:
    input_delta = plus_stats.mit_input_tau_mean - minus_stats.mit_input_tau_mean
    executed_delta = MIT_EXECUTION_SCALE * input_delta
    raw_delta = plus_stats.raw_mean - minus_stats.raw_mean
    div1000_delta = plus_stats.effort_div1000_mean - minus_stats.effort_div1000_mean
    fw18_delta = plus_stats.effort_fw18_mean - minus_stats.effort_fw18_mean
    if abs(input_delta) < 1e-9:
        raise ValueError("plus/minus MIT input torque difference is too small")
    alpha_div1000 = div1000_delta / input_delta
    alpha_fw18 = fw18_delta / input_delta
    feedback_per_executed_div1000 = div1000_delta / executed_delta
    feedback_per_executed_fw18 = fw18_delta / executed_delta
    raw_counts_per_input_nm = raw_delta / input_delta
    print(
        "auto summary: "
        f"plus_samples={plus_stats.samples}, minus_samples={minus_stats.samples}"
    )
    print(
        "tau_plus/tau_minus: "
        f"raw={plus_stats.raw_mean:+.3f}/{minus_stats.raw_mean:+.3f}, "
        "div1000_nm="
        f"{plus_stats.effort_div1000_mean:+.5f}/"
        f"{minus_stats.effort_div1000_mean:+.5f}, "
        "fw18_nm="
        f"{plus_stats.effort_fw18_mean:+.5f}/"
        f"{minus_stats.effort_fw18_mean:+.5f}"
    )
    print(
        "mit_input_tau_mean_encoded_sdk_nm: "
        f"plus={plus_stats.mit_input_tau_mean:+.6f}, "
        f"minus={minus_stats.mit_input_tau_mean:+.6f}, "
        f"diff={input_delta:+.6f}; "
        f"4x_executed_diff={executed_delta:+.6f}"
    )
    print(
        "feedback_delta: "
        f"raw_counts={raw_delta:+.3f}, "
        f"div1000_nm={div1000_delta:+.6f}, "
        f"fw18_nm={fw18_delta:+.6f}"
    )
    print(
        "alpha_hat for tau_feedback = alpha * "
        "(kp*(p_des-q)+kd*(0-dq)+tau_ff): "
        f"div1000_nm={alpha_div1000:+.5f}, "
        f"fw18_nm={alpha_fw18:+.5f}"
    )
    print(
        "feedback_per_4x_executed_hypothesis: "
        f"raw_counts_per_sdk_input_nm={raw_counts_per_input_nm:+.3f}, "
        f"div1000_nm={feedback_per_executed_div1000:+.5f}, "
        f"fw18_nm={feedback_per_executed_fw18:+.5f}"
    )


def main() -> int:
    args = _parse_args()
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.auto_mode == "position" and args.auto_offset_rad <= 0.0:
        raise ValueError("--auto-offset-rad must be positive")
    if args.auto_mode == "torque" and args.auto_t_ref_nm <= 0.0:
        raise ValueError("--auto-t-ref-nm must be positive for --auto-mode torque")
    if args.auto_hold_s <= 0.0:
        raise ValueError("--auto-hold-s must be positive")
    if args.auto_settle_s < 0.0:
        raise ValueError("--auto-settle-s must be non-negative")
    if args.auto_settle_s >= args.auto_hold_s:
        raise ValueError("--auto-settle-s must be smaller than --auto-hold-s")
    if args.auto_cycles <= 0:
        raise ValueError("--auto-cycles must be positive")
    if args.other_kp is None:
        args.other_kp = args.kp

    stop_requested = False

    def _request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        print(f"\nreceived signal {signum}; neutralizing and exiting...", file=sys.stderr)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    _check_no_controller_conflict(args)
    _print_command_scale(args)

    output_path = _make_output_path(args)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    from piper_sdk import C_PiperInterface_V2

    piper = C_PiperInterface_V2(args.can)
    print(f"connecting to {args.can}...")
    piper.ConnectPort()
    time.sleep(0.2)

    if not args.no_enable:
        print("enabling Piper...")
        _enable_piper(piper, args.enable_timeout_s)

    p_des = _joint_positions_rad(piper)
    last_q = list(p_des)
    target_idx = args.joint - 1
    kp = [float(args.other_kp)] * 6
    kd = [float(args.other_kd)] * 6
    t_ref = [float(args.other_t_ref)] * 6
    kp[target_idx] = float(args.kp)
    kd[target_idx] = float(args.kd)
    t_ref[target_idx] = float(args.t_ref)

    print("initial joint positions rad: " + ", ".join(f"{q:.5f}" for q in p_des))
    print(
        "streaming MIT: "
        f"can={args.can}, target_joint={args.joint}, kp={args.kp}, "
        f"kd={args.kd}, t_ref={args.t_ref}, rate={args.rate_hz} Hz"
    )
    print(f"logging CSV to {output_path}")
    print(
        f"automated check mode={args.auto_mode}. "
        "Keep the arm supported or mechanically constrained."
    )

    _enter_mit_mode(piper)

    period_s = 1.0 / args.rate_hz
    started = time.monotonic()
    next_tick = started
    next_print = started + args.print_every_s if args.print_every_s > 0 else math.inf
    next_mode_refresh = started + args.mode_refresh_s
    sample_idx = 0
    plus_stats = AutoPhaseStats("plus", 1)
    minus_stats = AutoPhaseStats("minus", -1)

    try:
        with output_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_csv_fields())
            writer.writeheader()
            for cycle_idx in range(args.auto_cycles):
                for phase, sign, stats in (
                    ("plus", 1, plus_stats),
                    ("minus", -1, minus_stats),
                ):
                    phase_started = time.monotonic()
                    p_des_phase = list(p_des)
                    t_ref_phase = list(t_ref)
                    command_offset_rad = 0.0
                    command_t_ref_offset_nm = 0.0
                    if args.auto_mode == "position":
                        command_offset_rad = sign * args.auto_offset_rad
                        p_des_phase[target_idx] += command_offset_rad
                        command_label = (
                            f"p_des_j{args.joint}="
                            f"{p_des_phase[target_idx]:+.5f} rad"
                        )
                    else:
                        command_t_ref_offset_nm = sign * args.auto_t_ref_nm
                        t_ref_phase[target_idx] += command_t_ref_offset_nm
                        command_label = (
                            f"t_ref_j{args.joint}="
                            f"{t_ref_phase[target_idx]:+.5f} Nm"
                        )
                    print(
                        f"auto cycle {cycle_idx + 1}/{args.auto_cycles}: "
                        f"{phase} {command_label}"
                    )
                    while not stop_requested:
                        now = time.monotonic()
                        elapsed = now - started
                        phase_t_s = now - phase_started
                        if phase_t_s >= args.auto_hold_s:
                            break

                        if args.mode_refresh_s > 0.0 and now >= next_mode_refresh:
                            _enter_mit_mode(piper)
                            next_mode_refresh = now + args.mode_refresh_s

                        _send_mit_frame_set(piper, p_des_phase, kp, kd, t_ref_phase)
                        q = _joint_positions_rad(piper)
                        vel, raw_effort = _high_speed(piper)
                        last_q = q
                        include_in_mean = phase_t_s >= args.auto_settle_s
                        mit_input_tau_nm = _mit_input_tau_nm(
                            target_idx=target_idx,
                            p_des=p_des_phase,
                            q=q,
                            vel=vel,
                            kp=kp,
                            kd=kd,
                            t_ref=t_ref_phase,
                        )
                        if include_in_mean:
                            stats.add(
                                args.joint,
                                raw_effort[target_idx],
                                mit_input_tau_nm,
                            )

                        row = _row(
                            args=args,
                            sample_idx=sample_idx,
                            t_s=elapsed,
                            p_des=p_des_phase,
                            q=q,
                            vel=vel,
                            raw_effort=raw_effort,
                            kp=kp,
                            kd=kd,
                            t_ref=t_ref_phase,
                            phase=phase,
                            phase_sign=sign,
                            phase_t_s=phase_t_s,
                            q0=p_des,
                            command_offset_rad=command_offset_rad,
                            command_t_ref_offset_nm=command_t_ref_offset_nm,
                            included_in_auto_mean=include_in_mean,
                        )
                        writer.writerow(row)
                        sample_idx += 1

                        if now >= next_print:
                            _print_row_status(row)
                            next_print = now + args.print_every_s

                        next_tick += period_s
                        sleep_s = next_tick - time.monotonic()
                        if sleep_s > 0.0:
                            time.sleep(sleep_s)
                        else:
                            next_tick = time.monotonic()
                    if stop_requested:
                        break
                if stop_requested:
                    break
    finally:
        print("neutralizing MIT gains...", file=sys.stderr)
        _neutralize(piper, last_q, args.neutralize_s, args.rate_hz)

    if plus_stats.samples > 0 and minus_stats.samples > 0:
        _print_auto_summary(
            args=args,
            plus_stats=plus_stats,
            minus_stats=minus_stats,
        )
    print(f"wrote {sample_idx} samples to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
