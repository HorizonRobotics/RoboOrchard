#!/usr/bin/env python3
"""Calibrate one Piper arm's MIT deflection compensation at a selected kp.

Hardware tool; see CALIBRATION.md for prerequisites and safe operation.
"""

from __future__ import annotations
import argparse
import datetime
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if __package__:
    from calibration.paths import DEFAULT_URDF
    from calibration.takeover_muxer import respawn_muxer, stop_muxer
else:
    from paths import DEFAULT_URDF
    from takeover_muxer import respawn_muxer, stop_muxer

DEFAULT_STORE = HERE / "deflection_calibrations.json"
DEFAULT_OUT_DIR = "/data/holobrain/deflection"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--kp",
        type=float,
        required=True,
        help="mit_kp to calibrate (the kp the arm will run at).",
    )
    parser.add_argument("--store", default=str(DEFAULT_STORE))
    parser.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    parser.add_argument("--urdf", default=DEFAULT_URDF)
    parser.add_argument(
        "--keep-muxer-stopped",
        action="store_true",
        help="Do not respawn the take_over muxer afterwards.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(
        args.out_dir, f"{args.side}_kp{args.kp:g}_{stamp}.csv"
    )

    muxer_argv = stop_muxer(args.side)
    try:
        measure_cmd = [
            sys.executable,
            str(HERE / "measure_deflection.py"),
            "--side",
            args.side,
            "--kp",
            f"{args.kp:g}",
            "--out",
            csv_path,
            "--yes",
        ]
        print("+", " ".join(measure_cmd), flush=True)
        subprocess.run(measure_cmd, check=True)

        fit_cmd = [
            sys.executable,
            str(HERE / "fit_deflection.py"),
            "--csv",
            csv_path,
            "--side",
            args.side,
            "--urdf",
            args.urdf,
            "--update-calibration",
            args.store,
        ]
        print("+", " ".join(fit_cmd), flush=True)
        subprocess.run(fit_cmd, check=True)
    finally:
        if muxer_argv is not None and not args.keep_muxer_stopped:
            respawn_muxer(muxer_argv)

    print(
        f"\nDone: kp={args.kp:g} ({args.side}) is now calibrated in "
        f"{args.store}. The controller accepts it on the next mit_kp "
        "change (e.g. set kp and Apply MIT Params)."
    )


if __name__ == "__main__":
    main()
