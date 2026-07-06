#!/usr/bin/env python3
"""Calibrate one arm's deflection compensation for a new mit_kp, end to end.

One command wrapping the full DEFLECTION_CALIBRATION.md workflow:

  1. Stop the side's take_over muxer (it holds a registered publisher on
     /robot/<side>/joint_cmd, which measure_deflection.py refuses to run
     alongside), remembering its exact command line.
  2. Run measure_deflection.py at the requested kp (comp off, safe
     default pose grid, ~10 min of arm motion — keep the e-stop in
     reach).
  3. Run fit_deflection.py --update-calibration so the fitted values
     land in the kp-indexed store the controllers read
     (deflection_calibrations.json). The new kp is accepted immediately.
  4. Respawn the muxer with its original command line.

Usage (inside the holobrain container, ROS sourced, control mode STOP):

    python3 /opt/roboorchard/gravity_id/calibrate_kp.py --side left --kp 30
"""

from __future__ import annotations

import argparse
import datetime
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_STORE = HERE / "deflection_calibrations.json"
DEFAULT_OUT_DIR = "/data/holobrain/deflection"
DEFAULT_URDF = "/data/holobrain/urdf/piper_x_description_dualarm_v2.urdf"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--kp", type=float, required=True,
        help="mit_kp to calibrate (the kp the arm will run at).",
    )
    parser.add_argument("--store", default=str(DEFAULT_STORE))
    parser.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    parser.add_argument("--urdf", default=DEFAULT_URDF)
    parser.add_argument(
        "--keep-muxer-stopped", action="store_true",
        help="Do not respawn the take_over muxer afterwards.",
    )
    return parser.parse_args()


def _find_muxer(side: str) -> tuple[int, list[str]] | None:
    """PID + argv of the side's take_over muxer, or None."""
    marker = f"robot_{side}_takeover_muxer"
    for pid_dir in Path("/proc").iterdir():
        if not pid_dir.name.isdigit():
            continue
        try:
            argv = (pid_dir / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        argv = [a.decode(errors="replace") for a in argv if a]
        if any(marker in a for a in argv) and not any(
            "calibrate_kp" in a for a in argv
        ):
            return int(pid_dir.name), argv
    return None


def _stop_muxer(side: str) -> list[str] | None:
    found = _find_muxer(side)
    if found is None:
        print(f"No take_over muxer running for side {side}.")
        return None
    pid, argv = found
    print(f"Stopping take_over muxer (pid {pid}) for the measurement...")
    os.kill(pid, signal.SIGTERM)
    for _ in range(50):
        if _find_muxer(side) is None:
            return argv
        time.sleep(0.1)
    raise SystemExit(f"take_over muxer (pid {pid}) did not exit.")


def _respawn_muxer(argv: list[str]) -> None:
    # The original --params-file lives in the container's /tmp and
    # normally outlives the process; if it was cleaned up, only a pane
    # restart can bring the muxer back.
    for i, arg in enumerate(argv):
        if arg == "--params-file" and i + 1 < len(argv):
            if not os.path.exists(argv[i + 1]):
                print(
                    "WARNING: muxer params file "
                    f"{argv[i + 1]} is gone; NOT respawning. Restart the "
                    "robot-control pane (restart_piper_gravity.sh) to "
                    "restore takeover."
                )
                return
    print("Respawning take_over muxer...")
    proc = subprocess.Popen(
        argv,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    time.sleep(2.0)
    if proc.poll() is not None:
        print(
            f"WARNING: respawned muxer exited (code {proc.returncode}). "
            "Restart the robot-control pane (restart_piper_gravity.sh) "
            "to restore takeover."
        )
    else:
        print(f"take_over muxer respawned (pid {proc.pid}).")


def main() -> None:
    args = _parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(
        args.out_dir, f"{args.side}_kp{args.kp:g}_{stamp}.csv"
    )

    muxer_argv = _stop_muxer(args.side)
    try:
        measure_cmd = [
            sys.executable, str(HERE / "measure_deflection.py"),
            "--side", args.side,
            "--kp", f"{args.kp:g}",
            "--out", csv_path,
            "--yes",
        ]
        print("+", " ".join(measure_cmd), flush=True)
        subprocess.run(measure_cmd, check=True)

        fit_cmd = [
            sys.executable, str(HERE / "fit_deflection.py"),
            "--csv", csv_path,
            "--side", args.side,
            "--urdf", args.urdf,
            "--update-calibration", args.store,
        ]
        print("+", " ".join(fit_cmd), flush=True)
        subprocess.run(fit_cmd, check=True)
    finally:
        if muxer_argv is not None and not args.keep_muxer_stopped:
            _respawn_muxer(muxer_argv)

    print(
        f"\nDone: kp={args.kp:g} ({args.side}) is now calibrated in "
        f"{args.store}. The controller accepts it on the next mit_kp "
        "change (e.g. set kp and Apply MIT Params)."
    )


if __name__ == "__main__":
    main()
