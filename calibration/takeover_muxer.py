#!/usr/bin/env python3
"""Stop and restore the per-side takeover muxer during calibration."""

from __future__ import annotations
import os
import signal
import subprocess
import time
from pathlib import Path

_HELPER_SCRIPTS = {
    "calibrate_kp.py",
    "calibrate_friction.py",
    "takeover_muxer.py",
}


def find_muxer(side: str) -> tuple[int, list[str]] | None:
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
        is_helper = any(Path(arg).name in _HELPER_SCRIPTS for arg in argv)
        if any(marker in arg for arg in argv) and not is_helper:
            return int(pid_dir.name), argv
    return None


def stop_muxer(side: str) -> list[str] | None:
    """SIGTERM the side's muxer; returns its argv for respawning."""
    found = find_muxer(side)
    if found is None:
        print(f"No take_over muxer running for side {side}.")
        return None
    pid, argv = found
    print(f"Stopping take_over muxer (pid {pid}) for the measurement...")
    os.kill(pid, signal.SIGTERM)
    for _ in range(50):
        if find_muxer(side) is None:
            return argv
        time.sleep(0.1)
    raise SystemExit(f"take_over muxer (pid {pid}) did not exit.")


def respawn_muxer(argv: list[str]) -> None:
    """Restart the muxer with the argv captured by :func:`stop_muxer`."""
    # The original --params-file lives in the container's /tmp and
    # normally outlives the process; if it was cleaned up, only a pane
    # restart can bring the muxer back.
    for i, arg in enumerate(argv):
        if arg == "--params-file" and i + 1 < len(argv):
            if not os.path.exists(argv[i + 1]):
                print(
                    "WARNING: muxer params file "
                    f"{argv[i + 1]} is gone; NOT respawning. Re-run "
                    "teleop/dagger.sh in the robot-control pane (it "
                    "replaces stale processes itself) to restore "
                    "takeover."
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
            "Re-run teleop/dagger.sh in the robot-control pane (it "
            "replaces stale processes itself) to restore takeover."
        )
    else:
        print(f"take_over muxer respawned (pid {proc.pid}).")
