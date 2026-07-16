#!/usr/bin/env python3
"""Characterize one Piper arm's Coulomb friction from triangle sweeps.

Hardware tool; see CALIBRATION.md for prerequisites and safe operation.
"""

from __future__ import annotations
import argparse
import csv
import datetime
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if __package__:
    from calibration.friction_fit import (
        CoulombFit,
        fit_coulomb,
        hysteresis_half_gap,
        tau_from_deflection,
    )
    from calibration.store import load_store, write_store
    from calibration.takeover_muxer import respawn_muxer, stop_muxer
else:
    from friction_fit import (
        CoulombFit,
        fit_coulomb,
        hysteresis_half_gap,
        tau_from_deflection,
    )
    from store import load_store, write_store
    from takeover_muxer import respawn_muxer, stop_muxer

DEFAULT_STORE = HERE / "deflection_calibrations.json"
DEFAULT_OUT_DIR = "/data/holobrain/friction"
DEFAULT_CENTER = [0.0, 1.0, -1.4, 0.0, 0.0, 0.0]
RESULT_SCHEMA_VERSION = 1

MEASUREMENT_PARAMS = {
    "mit_friction_compensation_enabled": False,
}

CSV_FIELDS = ["side", "joint", "speed", "t", "q_cmd", "q_meas", "v_sign"]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--joints",
        default="2,3",
        help="Comma-separated 1-based joints to sweep (default: 2,3 — "
        "the only joints with measurable Coulomb deflection).",
    )
    parser.add_argument(
        "--speeds",
        default="0.2,0.35,0.5",
        help="Comma-separated sweep speeds in rad/s; at least 2 needed "
        "for the v->0 extrapolation (default: 0.2,0.35,0.5).",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.35,
        help="Half-stroke around the center pose (rad).",
    )
    parser.add_argument(
        "--center",
        default="",
        help="Comma-separated 6-joint center pose (rad). Default is a "
        "table-safe pose bracketed by the deflection grid.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="Full triangle cycles per (joint, speed).",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=None,
        help="mit_kp to measure at; default keeps the controller's "
        "current kp. Must have a deflection entry in the store (the "
        "stiffness converts deflection to torque).",
    )
    parser.add_argument(
        "--calibration-store",
        "--store",
        dest="calibration_store",
        default=str(DEFAULT_STORE),
        help="Read-only deflection calibration store used for stiffness.",
    )
    parser.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    parser.add_argument(
        "--result-json",
        default="",
        help="Characterization result path (default: beside the raw CSV).",
    )
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--max-error",
        type=float,
        default=0.35,
        help="Abort if any joint deviates from its command by more "
        "than this (rad).",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.15,
        help="Max joint speed (rad/s) for ramps to/from the center.",
    )
    parser.add_argument(
        "--min-r2",
        type=float,
        default=0.8,
        help="Exclude joints below this fit quality from the candidate "
        "summary.",
    )
    parser.add_argument(
        "--max-tau",
        type=float,
        default=1.5,
        help="Clamp fitted Coulomb torque to this (Nm).",
    )
    parser.add_argument(
        "--keep-muxer-stopped",
        action="store_true",
        help="Do not respawn the take_over muxer afterwards.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive confirmation.",
    )
    return parser.parse_args()


def _parse_floats(text: str, what: str) -> list[float]:
    try:
        return [float(v) for v in text.split(",") if v.strip()]
    except ValueError as exc:
        raise SystemExit(f"bad {what} {text!r}: {exc}") from exc


def _stiffness_for_kp(store_path: str, side: str, kp: float) -> list[float]:
    """offset_stiffness of the store's deflection entry at this kp."""
    store = load_store(store_path)
    per_side = store.get(side)
    if not isinstance(per_side, dict):
        raise SystemExit(
            f"no deflection entries for side {side!r} in {store_path}"
        )
    for key, entry in per_side.items():
        if abs(float(key) - kp) < 1e-6:
            return [float(v) for v in entry["offset_stiffness"]]
    pretty = ", ".join(sorted(f"{float(k):g}" for k in per_side))
    raise SystemExit(
        f"No deflection calibration for mit_kp={kp:g} (side {side}) in "
        f"{store_path}; its offset_stiffness is needed to convert the "
        f"Coulomb deflection to a torque. Calibrated kp values: "
        f"[{pretty}]. Run calibrate_kp.py for this kp first."
    )


def _characterization_result(
    side: str,
    kp: float,
    taus: dict[int, float],
    fits: dict[int, CoulombFit],
    stiffness: list[float],
    csv_path: str,
    min_r2: float,
    max_tau: float,
) -> dict:
    """Build a standalone, non-deployable characterization result."""
    scale = [0.0] * 6
    for joint, tau in taus.items():
        scale[joint - 1] = round(tau, 4)
    fit_entries = {}
    for joint in sorted(fits):
        fit = fits[joint]
        tau, was_clamped = tau_from_deflection(
            fit.intercept_rad, stiffness[joint - 1], max_tau
        )
        fit_entries[str(joint)] = {
            "speeds_rad_s": fit.speeds,
            "half_gap_rad": [round(g, 6) for g in fit.half_gaps],
            "intercept_rad": round(fit.intercept_rad, 6),
            "slope_rad_per_rad_s": round(fit.slope_rad_per_rad_s, 6),
            "r2": round(fit.r2, 4),
            "stiffness_nm_per_rad": stiffness[joint - 1],
            "tau_c_nm": round(tau, 4),
            "clamped": was_clamped,
            "passes_quality_gate": joint in taus,
        }
    return {
        "schema_version": RESULT_SCHEMA_VERSION,
        "kind": "piper_coulomb_friction_characterization",
        "deployment_status": "characterization_only",
        "side": side,
        "kp": kp,
        "measured": datetime.date.today().isoformat(),
        "method": "triangle sweep, matched-position half-gap, v->0 intercept",
        "source_csv": os.path.basename(csv_path),
        "candidate_scale_nm": scale,
        "quality_gate": {"min_r2": min_r2, "max_tau_nm": max_tau},
        "fits": fit_entries,
    }


def _validate_result_path(result_path: str, calibration_store: str) -> None:
    """Reject attempts to overwrite the runtime calibration store."""
    if Path(result_path).resolve() == Path(calibration_store).resolve():
        raise SystemExit(
            "--result-json must not overwrite the runtime calibration store"
        )


def main() -> None:  # noqa: C901
    args = _parse_args()
    joints = [int(j) for j in args.joints.split(",") if j.strip()]
    if not joints or any(j < 1 or j > 6 for j in joints):
        raise SystemExit(
            f"--joints must be 1-based joint numbers, got {args.joints!r}"
        )
    speeds = _parse_floats(args.speeds, "--speeds")
    if len(set(speeds)) < 2 or any(s <= 0 for s in speeds):
        raise SystemExit(
            "--speeds needs at least 2 distinct positive "
            "values for the v->0 extrapolation"
        )
    center = (
        _parse_floats(args.center, "--center")
        if args.center
        else list(DEFAULT_CENTER)
    )
    if len(center) != 6:
        raise SystemExit("--center needs 6 joint values")
    if not 0.05 <= args.amplitude <= 0.6:
        raise SystemExit("--amplitude must be in [0.05, 0.6] rad")

    # Import rclpy only here so --help works without a ROS environment.
    import rclpy

    if __package__:
        from calibration.measure_deflection import _DeflectionMeasurer
    else:
        from measure_deflection import _DeflectionMeasurer

    print(
        f"side={args.side} joints={joints} speeds={speeds} "
        f"amplitude={args.amplitude:g} cycles={args.cycles}"
    )
    print(f"center pose: {center}")
    if not args.yes:
        reply = input(
            "Arm will sweep the joints above with friction comp OFF. "
            "Teleop publishers stopped? E-stop in reach? [y/N] "
        )
        if reply.strip().lower() != "y":
            raise SystemExit("aborted")

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out_dir, f"{args.side}_friction_{stamp}.csv")
    result_path = args.result_json or os.path.join(
        args.out_dir, f"{args.side}_friction_{stamp}_fit.json"
    )
    _validate_result_path(result_path, args.calibration_store)

    muxer_argv = stop_muxer(args.side)
    rclpy.init()
    node = _DeflectionMeasurer(args.side)
    exit_code = 0
    try:
        node.assert_sole_publisher()
        home = node.wait_for_state()
        print(f"start pose: {[round(v, 3) for v in home]}")

        saved_names = list(MEASUREMENT_PARAMS) + ["mit_kp"]
        saved = node.get_params(saved_names)
        print(f"saved controller params: {saved}")
        kp = args.kp if args.kp is not None else saved.get("mit_kp")
        if not kp:
            raise SystemExit(
                "could not read mit_kp from the controller; "
                "pass --kp explicitly"
            )
        stiffness = _stiffness_for_kp(args.calibration_store, args.side, kp)
        for j in joints:
            if stiffness[j - 1] <= 0.0:
                raise SystemExit(
                    f"J{j} has no offset_stiffness in the {args.side}/"
                    f"{kp:g} deflection entry; cannot convert its "
                    f"deflection to a torque. Sweep only calibrated "
                    f"joints (got --joints {args.joints})."
                )
        forced = dict(MEASUREMENT_PARAMS)
        if args.kp is not None:
            forced["mit_kp"] = args.kp
        node.set_params(forced)

        rows: list[dict] = []
        fits: dict[int, CoulombFit] = {}
        try:
            node.ramp_to(center, args.rate, args.max_speed)
            node.hold(center, args.rate, 1.5, args.max_error)
            for joint in joints:
                jidx = joint - 1
                lo = center[jidx] - args.amplitude
                hi = center[jidx] + args.amplitude
                half_gaps = []
                for speed in speeds:
                    print(f"J{joint} sweep at {speed:g} rad/s...")
                    q_cmd_rec: list[float] = []
                    q_meas_rec: list[float] = []
                    sign_rec: list[int] = []
                    q = list(center)
                    direction = 1
                    step = speed / args.rate
                    # one warm-up half-cycle + recorded cycles
                    strokes = 1 + 2 * args.cycles
                    t0 = time.monotonic()
                    for stroke in range(strokes):
                        while True:
                            q[jidx] += direction * step
                            if q[jidx] >= hi:
                                q[jidx] = hi
                            elif q[jidx] <= lo:
                                q[jidx] = lo
                            node.publish_cmd(q)
                            node._check_error(q, args.max_error)
                            node._spin_sleep(1.0 / args.rate)
                            if stroke > 0 and node.latest is not None:
                                q_cmd_rec.append(q[jidx])
                                q_meas_rec.append(node.latest[jidx])
                                sign_rec.append(direction)
                                rows.append(
                                    {
                                        "side": args.side,
                                        "joint": joint,
                                        "speed": speed,
                                        "t": round(time.monotonic() - t0, 4),
                                        "q_cmd": round(q[jidx], 6),
                                        "q_meas": round(node.latest[jidx], 6),
                                        "v_sign": direction,
                                    }
                                )
                            if q[jidx] in (lo, hi):
                                direction = -direction
                                break
                    gap = hysteresis_half_gap(
                        q_cmd_rec, q_meas_rec, sign_rec, lo, hi
                    )
                    half_gaps.append(gap)
                    print(
                        f"  half-gap {gap * 1000:+.3f} mrad "
                        f"({len(q_cmd_rec)} samples)"
                    )
                    # park back at the center before the next sweep
                    node.ramp_to(center, args.rate, args.max_speed)
                fits[joint] = fit_coulomb(speeds, half_gaps)
        finally:
            try:
                print("returning to start pose...")
                node.ramp_to(center, args.rate, args.max_speed)
                node.ramp_to(home, args.rate, args.max_speed)
            finally:
                print(f"restoring controller params: {saved}")
                # mit_kp first: enabling comp is rejected while an
                # uncalibrated kp is active.
                order = sorted(
                    saved, key=lambda k: 0 if k.startswith("mit_k") else 1
                )
                node.set_params(
                    {k: saved[k] for k in order if saved[k] is not None}
                )
                if rows:
                    with open(csv_path, "w", newline="") as fh:
                        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
                        writer.writeheader()
                        writer.writerows(rows)
                    print(f"wrote {len(rows)} samples -> {csv_path}")

        if not fits:
            raise SystemExit("no sweeps completed")
        taus: dict[int, float] = {}
        print(f"\nfit results (kp={kp:g}):")
        for joint, fit in fits.items():
            tau, was_clamped = tau_from_deflection(
                fit.intercept_rad, stiffness[joint - 1], args.max_tau
            )
            print(
                f"  J{joint}: intercept {fit.intercept_rad * 1000:+.3f} "
                f"mrad x k_eff {stiffness[joint - 1]:g} Nm/rad -> "
                f"tau_c {tau:.3f} Nm (r2={fit.r2:.3f}"
                + (", CLAMPED" if was_clamped else "")
                + ")"
            )
            if fit.r2 < args.min_r2:
                print(
                    f"  J{joint}: r2 {fit.r2:.3f} < --min-r2 "
                    f"{args.min_r2}; excluded from candidate summary. "
                    "Half-gap "
                    "vs speed is not linear — re-run with more cycles "
                    "or different speeds."
                )
                continue
            if joint == 2 and tau > 0.55:
                print(
                    "  note: J2 historically saturates near 0.55 Nm — "
                    "the residual reversal error there is backlash, "
                    "which torque feedforward cannot remove; higher "
                    "values risk reversal overshoot."
                )
            taus[joint] = tau

        result = _characterization_result(
            args.side,
            kp,
            taus,
            fits,
            stiffness,
            csv_path,
            args.min_r2,
            args.max_tau,
        )
        write_store(result_path, result)
        print(f"characterization result -> {result_path}")
        print(
            "No controller parameters or runtime calibration were persisted."
        )
        if not taus:
            raise SystemExit(
                "no joint passed the fit quality gate; raw and fitted "
                "characterization results were retained"
            )
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f"\nstopping: {exc}")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if muxer_argv is not None and not args.keep_muxer_stopped:
            respawn_muxer(muxer_argv)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
