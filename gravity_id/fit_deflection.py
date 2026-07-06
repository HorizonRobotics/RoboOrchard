#!/usr/bin/env python3
"""Fit MIT offset-stiffness / deflection-table params (offline half).

Consumes the CSV written by ``measure_deflection.py`` and produces the two
controller parameters used by the kp-offset gravity-compensation path:

  * ``mit_gravity_compensation_offset_stiffness``  — per-joint scalar
    stiffness (Nm/rad), used for joints whose stiffness is load-independent
    and as fallback when the deflection table is disabled.
  * ``mit_gravity_compensation_deflection_table``  — JSON knot table
    ``{"<joint>": [[|tau| Nm, |defl| rad], ...]}`` for joints whose
    stiffness is load-dependent (J2 stiffens with load). The controller
    gives a non-empty table precedence over the scalar.

Method (matches the 2026-07-02 left-arm calibration that produced
stiffness [0, 207, 418, 93, 0, 0] and the J2 table):

  1. For each settled pose, gravity torque tau_j = RNEA at the COMMANDED
     pose (same evaluation point the controller uses at runtime) via the
     pure-python URDF summation in ``urdf_gravity_check.py``.
  2. Deflection s_j = q_cmd_j - q_meas_j. Rows measured with approach
     'above' and 'below' at the same pose are averaged so stiction
     cancels; half their difference is reported as the stiction band.
  3. Per joint: one (|tau|, |s|) knot per pose. If the secant stiffness
     tau/s varies with load beyond ``--spread`` the joint gets a
     deflection table; the scalar stiffness is the mean of the two
     lowest-load secants (table joints) or the least-squares
     through-origin fit (constant-stiffness joints).

Runs anywhere numpy is available (holobrain container works):

    python3 gravity_id/fit_deflection.py \
        --csv /data/holobrain/deflection/left_kp40.csv --side left \
        --urdf /data/holobrain/urdf/piper_x_description_dualarm_v2.urdf

Prints a diagnostic report plus ready-to-paste ``ros2 param set`` lines.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from urdf_gravity_check import gravity_torques  # noqa: E402

DEFAULT_URDF = "/data/holobrain/urdf/piper_x_description_dualarm_v2.urdf"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--csv", nargs="+", required=True,
        help="measure_deflection.py output CSV(s); multiple files are "
        "pooled (must be same side and kp).",
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--urdf", default=DEFAULT_URDF,
        help="Dual-arm URDF used by the deployed compensator.",
    )
    parser.add_argument(
        "--joints", default="2,3,4",
        help="1-based joints to fit (default: gravity-loaded 2,3,4).",
    )
    parser.add_argument(
        "--min-torque", type=float, default=0.5,
        help="Ignore knots with |tau| below this (Nm).",
    )
    parser.add_argument(
        "--min-deflection", type=float, default=0.003,
        help="Ignore knots with |deflection| below this (rad); smaller "
        "values are inside encoder/stiction noise.",
    )
    parser.add_argument(
        "--spread", type=float, default=1.2,
        help="Emit a deflection table for a joint when max/min secant "
        "stiffness exceeds this ratio (load-dependent stiffness).",
    )
    parser.add_argument(
        "--update-calibration", default="", metavar="JSON",
        help="Write the fitted values into this kp-indexed calibration "
        "store (gravity_id/deflection_calibrations.json), creating or "
        "replacing the <side>/<kp> entry. This is how a new kp becomes "
        "accepted by the follower controllers.",
    )
    return parser.parse_args()


def _load_rows(paths: list[str], side: str) -> tuple[float, list[dict]]:
    rows: list[dict] = []
    kps = set()
    for path in paths:
        with open(path) as fh:
            for row in csv.DictReader(fh):
                if row["side"] != side:
                    raise SystemExit(
                        f"{path} row has side={row['side']!r}, "
                        f"expected {side!r}"
                    )
                kps.add(float(row["kp"]))
                rows.append(row)
    if not rows:
        raise SystemExit("no rows")
    if len(kps) > 1:
        raise SystemExit(
            f"mixed mit_kp values {sorted(kps)}; stiffness depends on kp, "
            "fit one kp at a time"
        )
    return kps.pop(), rows


def _merge_approaches(
    rows: list[dict],
) -> list[dict]:
    """Average 'above'/'below' rows per pose; report stiction band."""
    by_pose: dict[tuple, list[dict]] = {}
    for row in rows:
        key = tuple(round(float(row[f"q_cmd{j}"]), 4) for j in range(1, 7))
        by_pose.setdefault(key, []).append(row)
    merged = []
    for key, group in sorted(by_pose.items()):
        q_cmd = [float(group[0][f"q_cmd{j}"]) for j in range(1, 7)]
        q_meas = [
            sum(float(r[f"q{j}"]) for r in group) / len(group)
            for j in range(1, 7)
        ]
        band = [0.0] * 6
        if len(group) > 1:
            band = [
                max(float(r[f"q{j}"]) for r in group)
                - min(float(r[f"q{j}"]) for r in group)
                for j in range(1, 7)
            ]
        merged.append({
            "pose_idx": group[0]["pose_idx"],
            "q_cmd": q_cmd,
            "q_meas": q_meas,
            "band": band,
            "n_approaches": len(group),
        })
    return merged


def main() -> None:
    args = _parse_args()
    joints = [int(j) for j in args.joints.split(",")]
    prefix = f"{args.side}_joint"
    kp, rows = _load_rows(args.csv, args.side)
    poses = _merge_approaches(rows)

    print(f"side={args.side} kp={kp:g} urdf={args.urdf}")
    print(f"{len(rows)} rows -> {len(poses)} poses "
          f"(approaches averaged)\n")

    # Per-joint knots: (|tau|, |deflection|, pose_idx, band)
    knots: dict[int, list[tuple[float, float, str, float]]] = {
        j: [] for j in joints
    }
    print("pose  " + "".join(
        f"| J{j}: tau(Nm) sag(rad) k(Nm/rad) " for j in joints
    ))
    for pose in poses:
        tau = gravity_torques(args.urdf, pose["q_cmd"], prefix=prefix)
        line = f"P{pose['pose_idx']:>3} "
        for j in joints:
            t, s = tau[j - 1], pose["q_cmd"][j - 1] - pose["q_meas"][j - 1]
            b = pose["band"][j - 1]
            sec = abs(t) / abs(s) if abs(s) > 1e-9 else float("inf")
            line += f"| {t:+7.2f} {s:+8.4f} {sec:8.1f} "
            if abs(t) < args.min_torque or abs(s) < args.min_deflection:
                continue
            if t * s < 0:
                print(
                    f"WARNING P{pose['pose_idx']} J{j}: deflection sign "
                    f"opposes gravity torque (tau={t:+.2f}, s={s:+.4f}); "
                    "knot dropped — check for contact or a second "
                    "publisher."
                )
                continue
            if b / 2 > 0.3 * abs(s):
                print(
                    f"note P{pose['pose_idx']} J{j}: stiction band/2 "
                    f"({b / 2:.4f}) is >30% of sag ({abs(s):.4f}); knot "
                    "kept but noisy."
                )
            knots[j].append((abs(t), abs(s), pose["pose_idx"], b))
        print(line)
    print()

    stiffness = [0.0] * 6
    tables: dict[str, list[list[float]]] = {}
    for j in joints:
        pts = sorted(knots[j])
        if len(pts) < 2:
            print(f"J{j}: only {len(pts)} usable knot(s); skipped "
                  "(no stiffness emitted, controller falls back to kp).")
            continue
        secants = [t / s for t, s, _, _ in pts]
        spread = max(secants) / min(secants)
        ls_fit = (
            sum(t * s for t, s, _, _ in pts)
            / sum(s * s for _, s, _, _ in pts)
        )
        print(f"J{j}: {len(pts)} knots, secant k = "
              f"{[round(k, 1) for k in secants]}, spread x{spread:.2f}, "
              f"LS-through-origin k = {ls_fit:.1f} Nm/rad")
        if spread > args.spread and len(pts) >= 3:
            # Load-dependent: emit a table; enforce strictly increasing.
            table: list[list[float]] = []
            for t, s, pidx, _ in pts:
                t_r, s_r = round(t, 2), round(s, 4)
                if table and (t_r <= table[-1][0] or s_r <= table[-1][1]):
                    print(f"  knot ({t_r}, {s_r}) from P{pidx} not "
                          "strictly increasing; merged/dropped.")
                    continue
                table.append([t_r, s_r])
            tables[str(j)] = table
            # Fallback scalar: mean of the two lowest-load secants (the
            # regime typical teleop poses live in).
            stiffness[j - 1] = round((secants[0] + secants[1]) / 2)
            print(f"  -> deflection table ({len(table)} knots); scalar "
                  f"fallback {stiffness[j - 1]:g} (low-load secants).")
        else:
            stiffness[j - 1] = round(ls_fit)
            print(f"  -> constant stiffness {stiffness[j - 1]:g} Nm/rad.")
    print()

    node = f"/robot/{args.side}/robot_{args.side}_controller"
    stiff_str = "[" + ", ".join(f"{v:.1f}" for v in stiffness) + "]"
    print("Apply with (values measured at mit_kp="
          f"{kp:g} — re-measure if deployment kp changes):\n")
    print(f'ros2 param set {node} '
          f'mit_gravity_compensation_offset_stiffness "{stiff_str}"')
    if tables:
        table_json = json.dumps(tables, separators=(",", ":"))
        escaped = table_json.replace('"', '\\"')
        print(f'ros2 param set {node} '
              f'mit_gravity_compensation_deflection_table "\'{escaped}\'"')
    else:
        print(f'ros2 param set {node} '
              f'mit_gravity_compensation_deflection_table "\'\'"'
              "  # no load-dependent joints found")

    if args.update_calibration:
        _update_calibration_store(
            args.update_calibration, args.side, kp, stiffness, tables,
            sources=args.csv,
        )


def _update_calibration_store(
    path: str,
    side: str,
    kp: float,
    stiffness: list[float],
    tables: dict[str, list[list[float]]],
    sources: list[str],
) -> None:
    """Write the fitted values into the kp-indexed calibration store.

    The store (gravity_id/deflection_calibrations.json) is what the
    follower controllers read via
    ``mit_gravity_compensation_calibration_file``; adding an entry here
    is what makes a new kp value accepted at runtime.
    """
    import datetime
    import os

    store: dict = {}
    if os.path.exists(path):
        with open(path) as f:
            store = json.load(f)
    key = f"{kp:g}"
    entry = {
        "offset_stiffness": stiffness,
        "deflection_table": tables,
        "measured": datetime.date.today().isoformat(),
        "source": " + ".join(os.path.basename(c) for c in sources),
    }
    replaced = key in store.get(side, {})
    store.setdefault(side, {})[key] = entry
    with open(path, "w") as f:
        json.dump(store, f, indent=2)
        f.write("\n")
    action = "Replaced" if replaced else "Added"
    print(f"\n{action} calibration entry {side}/kp={key} in {path}. "
          "Controllers pick it up on restart or on the next mit_kp "
          "change.")


if __name__ == "__main__":
    main()
