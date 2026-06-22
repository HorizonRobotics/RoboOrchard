#!/usr/bin/env python3
"""Log Piper MIT static stiffness data for one joint.

This script intentionally bypasses the RoboOrchard ROS controller. Run it only
when no other process is commanding the same Piper CAN interface.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

from piper_sdk import C_PiperInterface_V2


RAD_PER_MDEG = math.pi / 180000.0
DEFAULT_OUT_DIR = "/data/holobrain/mit_static_stiffness"


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
        help="Joint to manually displace and analyze.",
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
        "--duration-s",
        type=float,
        default=0.0,
        help="Run duration. 0 means run until Ctrl-C.",
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
        "p_des_rad",
        "q_rad",
        "delta_rad",
        "vel_rad_s",
        "raw_effort",
        "effort_nm_div1000",
        "effort_nm_fw18_j123",
        "expected_tau_kp_nm",
        "expected_tau_4kp_nm",
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
) -> dict[str, Any]:
    target_idx = args.joint - 1
    delta = [q[idx] - p_des[idx] for idx in range(6)]
    target_delta = delta[target_idx]
    target_raw_effort = raw_effort[target_idx]
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
        "p_des_rad": p_des[target_idx],
        "q_rad": q[target_idx],
        "delta_rad": target_delta,
        "vel_rad_s": vel[target_idx],
        "raw_effort": target_raw_effort,
        "effort_nm_div1000": _effort_nm_div1000(target_raw_effort),
        "effort_nm_fw18_j123": _effort_nm_fw18(args.joint, target_raw_effort),
        "expected_tau_kp_nm": -args.kp * target_delta,
        "expected_tau_4kp_nm": -4.0 * args.kp * target_delta,
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


def main() -> int:
    args = _parse_args()
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.other_kp is None:
        args.other_kp = args.kp

    stop_requested = False

    def _request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        print(f"\nreceived signal {signum}; neutralizing and exiting...", file=sys.stderr)

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    output_path = _make_output_path(args)
    output_path.parent.mkdir(parents=True, exist_ok=True)

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
    print("manually displace the target joint in small steps; Ctrl-C to stop.")

    _enter_mit_mode(piper)

    period_s = 1.0 / args.rate_hz
    started = time.monotonic()
    next_tick = started
    next_print = started + args.print_every_s if args.print_every_s > 0 else math.inf
    next_mode_refresh = started + args.mode_refresh_s
    sample_idx = 0

    try:
        with output_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_csv_fields())
            writer.writeheader()
            while not stop_requested:
                now = time.monotonic()
                elapsed = now - started
                if args.duration_s > 0.0 and elapsed >= args.duration_s:
                    break

                if args.mode_refresh_s > 0.0 and now >= next_mode_refresh:
                    _enter_mit_mode(piper)
                    next_mode_refresh = now + args.mode_refresh_s

                _send_mit_frame_set(piper, p_des, kp, kd, t_ref)
                q = _joint_positions_rad(piper)
                vel, raw_effort = _high_speed(piper)
                last_q = q

                row = _row(
                    args=args,
                    sample_idx=sample_idx,
                    t_s=elapsed,
                    p_des=p_des,
                    q=q,
                    vel=vel,
                    raw_effort=raw_effort,
                    kp=kp,
                    kd=kd,
                    t_ref=t_ref,
                )
                writer.writerow(row)
                sample_idx += 1

                if now >= next_print:
                    print(
                        "t={t_s:.2f}s delta={delta:+.5f} rad "
                        "vel={vel:+.4f} rad/s raw_effort={raw} "
                        "effort/1000={eff:+.4f} Nm fw18={fw:+.4f} Nm".format(
                            t_s=elapsed,
                            delta=float(row["delta_rad"]),
                            vel=float(row["vel_rad_s"]),
                            raw=row["raw_effort"],
                            eff=float(row["effort_nm_div1000"]),
                            fw=float(row["effort_nm_fw18_j123"]),
                        ),
                        flush=True,
                    )
                    next_print = now + args.print_every_s

                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
    finally:
        print("neutralizing MIT gains...", file=sys.stderr)
        _neutralize(piper, last_q, args.neutralize_s, args.rate_hz)

    print(f"wrote {sample_idx} samples to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
