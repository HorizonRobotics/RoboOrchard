#!/usr/bin/env python3
"""Measure Piper MIT static torque-to-deflection data (online half).

Commands a grid of static poses to one follower arm with gravity/friction
compensation DISABLED and mit_kp forced to the operating value, waits for
each pose to settle, and records the settled joint positions. The offline
companion (``fit_deflection.py``) converts the CSV this script writes into
the ``mit_gravity_compensation_offset_stiffness`` and
``mit_gravity_compensation_deflection_table`` controller parameters.

Physics: with compensation off, the firmware's internal loop holds the
joint near the command with a static error (sag). At equilibrium the
firmware torque equals the gravity torque, so each settled pose yields one
(gravity torque, deflection) sample of the firmware's effective stiffness
at the chosen mit_kp. The stiffness depends on mit_kp, so re-run this
whenever the deployment kp changes.

Run INSIDE the holobrain container with the ROS environment sourced:

    docker exec -it holobrain bash
    source /opt/ros/humble/setup.bash
    python3 /opt/roboorchard/gravity_id/measure_deflection.py \
        --side left --kp 40 --out /data/holobrain/deflection/left_kp40.csv

PREREQUISITES (same as kp_offset_consistency_check.md):
  * The ``single_ctrl`` controllers must be running.
  * Stop every other publisher on /robot/<side>/joint_cmd first
    (take_over muxer, aloha_orchestrator, inference app/policy). The
    script refuses to start if it detects an existing publisher —
    duplicate joint_cmd publishers caused the kd-buzz incident.
  * Keep the e-stop within reach; the arm moves through extended,
    gravity-loaded poses.

The script saves the controller parameters it touches and restores them
on exit (including Ctrl-C), and ramps the arm back to its starting pose.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Default pose grid. Chosen so J2 gravity torque spans ~1.1..9.7 Nm (both
# signs), J3 spans ~1.1..6.2 Nm and J4 ~0.1..1.4 Nm on the stock dual-arm
# URDF. The two highest/lowest-reach poses carry a wrist curl (q5=1.3)
# that lifts the gripper tip: without it, their "above" approach
# pre-poses put link7 below the table plane once low-kp sag (~0.045 rad
# on J2 at 10 Nm) is added — measured strike at kp=25 on 2026-07-06.
# With the curl and the 0.10 rad approach offset, every pose and
# pre-pose keeps the GRIPPER TIP (not just the wrist) >= 0.075 m above
# the base plane including sag, and stays inside x [-0.49, +0.62].
# Valid for both arms (the dual-arm URDF right chain is a pure
# translation of the left).
DEFAULT_POSES = [
    [0.0, 0.6, -1.4, 0.0, 0.0, 0.0],
    [0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
    [0.0, 1.25, -1.0, 0.0, 0.0, 0.0],
    [0.0, 1.57, -1.4, 0.0, 0.0, 0.0],
    [0.0, 2.0, -1.4, 0.0, 1.3, 0.0],
    [0.0, 1.2, -0.5, 0.0, 1.3, 0.0],
    [0.0, 0.9, -1.8, 0.0, 0.0, 0.0],
    [0.0, 1.0, -2.2, 0.0, 0.0, 0.0],
]

# Controller parameters forced during measurement (compensation must be
# fully off so the measured sag is the raw firmware compliance).
MEASUREMENT_PARAMS = {
    "mit_gravity_compensation_enabled": False,
    "mit_friction_compensation_enabled": False,
}

CSV_FIELDS = (
    ["side", "kp", "pose_idx", "approach", "n_samples"]
    + [f"q_cmd{i}" for i in range(1, 7)]
    + [f"q{i}" for i in range(1, 7)]
    + [f"std{i}" for i in range(1, 7)]
    + [f"eff{i}" for i in range(1, 7)]
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--kp", type=float, required=True,
        help="mit_kp to measure at (the deployment kp).",
    )
    parser.add_argument(
        "--kd", type=float, default=None,
        help="Optional mit_kd override during measurement.",
    )
    parser.add_argument(
        "--out", required=True, help="Output CSV path (container-visible)."
    )
    parser.add_argument(
        "--poses", default="",
        help="Optional pose file: one pose per line, 6 comma-separated "
        "joint radians. Default: built-in 8-pose grid.",
    )
    parser.add_argument(
        "--approach", choices=("both", "above", "below", "none"),
        default="both",
        help="Approach direction(s) per pose. 'both' measures each pose "
        "twice so stiction cancels in the offline average (default).",
    )
    parser.add_argument(
        "--approach-offset", type=float, default=0.10,
        help="Radians added to J2/J3 for the approach pre-pose. 0.10 "
        "clears the table on the default grid's high-torque pre-poses "
        "at low kp while still dominating the stiction band "
        "(~0.003 rad) and sag (~0.034 rad).",
    )
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--max-speed", type=float, default=0.15,
        help="Max joint speed (rad/s) for ramps between poses.",
    )
    parser.add_argument(
        "--max-error", type=float, default=0.35,
        help="Abort if any joint deviates from its command by more than "
        "this (rad) while holding.",
    )
    parser.add_argument(
        "--max-std", type=float, default=0.002,
        help="Reject a sample window whose per-joint std exceeds this "
        "(not settled); the pose is retried once.",
    )
    parser.add_argument(
        "--yes", action="store_true",
        help="Skip the interactive pose-grid confirmation.",
    )
    return parser.parse_args()


def _load_poses(path: str) -> list[list[float]]:
    if not path:
        return [list(p) for p in DEFAULT_POSES]
    poses = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = [float(v) for v in line.split(",")]
            if len(vals) != 6:
                raise SystemExit(f"pose line needs 6 values: {line!r}")
            poses.append(vals)
    if not poses:
        raise SystemExit(f"no poses in {path}")
    return poses


class _DeflectionMeasurer(Node):
    def __init__(self, side: str) -> None:
        super().__init__(f"deflection_measurer_{side}")
        self.side = side
        self.cmd_topic = f"/robot/{side}/joint_cmd"
        self.state_topic = f"/puppet/joint_{side}"
        self.ctrl_node = f"/robot/{side}/robot_{side}_controller"
        self.latest: list[float] | None = None
        self.latest_effort: list[float] | None = None
        self.samples: list[list[float]] = []
        self.effort_samples: list[list[float]] = []
        self.sampling = False
        self.create_subscription(
            JointState, self.state_topic, self._on_state, 50
        )
        self._pub = None
        self._set_cli = self.create_client(
            SetParameters, f"{self.ctrl_node}/set_parameters"
        )
        self._get_cli = self.create_client(
            GetParameters, f"{self.ctrl_node}/get_parameters"
        )

    # -- state ------------------------------------------------------------
    def _on_state(self, msg: JointState) -> None:
        if len(msg.position) < 6:
            return
        self.latest = [float(p) for p in msg.position[:6]]
        if len(msg.effort) >= 6:
            self.latest_effort = [float(e) for e in msg.effort[:6]]
        if self.sampling:
            self.samples.append(list(self.latest))
            if self.latest_effort is not None:
                self.effort_samples.append(list(self.latest_effort))

    def wait_for_state(self, timeout: float = 5.0) -> list[float]:
        deadline = time.monotonic() + timeout
        while self.latest is None:
            if time.monotonic() > deadline:
                raise SystemExit(
                    f"no JointState on {self.state_topic} within {timeout}s"
                )
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.latest)

    # -- publisher safety --------------------------------------------------
    def assert_sole_publisher(self) -> None:
        count = self.count_publishers(self.cmd_topic)
        if count > 0:
            raise SystemExit(
                f"{count} publisher(s) already on {self.cmd_topic}. Stop "
                "take_over/aloha_orchestrator/inference app first "
                "(duplicate joint_cmd publishers caused the kd-buzz "
                "incident)."
            )
        self._pub = self.create_publisher(JointState, self.cmd_topic, 10)

    def publish_cmd(self, q6: list[float]) -> None:
        msg = JointState()
        msg.position = [float(v) for v in q6] + [0.0]  # 7th = gripper closed
        self._pub.publish(msg)

    # -- parameters ---------------------------------------------------------
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

    # -- motion --------------------------------------------------------------
    def _spin_sleep(self, duration: float) -> None:
        """Sleep ``duration`` while still processing subscription callbacks."""
        end = time.monotonic() + duration
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.02))

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

    def hold(
        self,
        target: list[float],
        rate: float,
        duration: float,
        max_error: float,
    ) -> None:
        """Hold ``target`` for ``duration`` without sampling."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_cmd(target)
            self._check_error(target, max_error)
            self._spin_sleep(1.0 / rate)

    def hold_and_sample(
        self,
        target: list[float],
        rate: float,
        settle: float,
        seconds: float,
        max_error: float,
    ) -> tuple[list[float], list[float], list[float], int]:
        """Hold ``target``, settle, then sample; returns (mean, std, eff, n)."""
        self.hold(target, rate, settle, max_error)
        self.samples, self.effort_samples = [], []
        self.sampling = True
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_cmd(target)
            self._check_error(target, max_error)
            self._spin_sleep(1.0 / rate)
        self.sampling = False
        samples = self.samples
        if len(samples) < 10:
            raise SystemExit(
                f"only {len(samples)} samples on {self.state_topic}"
            )
        mean = [
            statistics.fmean(s[j] for s in samples) for j in range(6)
        ]
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

    def _check_error(self, target: list[float], max_error: float) -> None:
        if self.latest is None:
            return
        for j, (cmd, meas) in enumerate(zip(target, self.latest)):
            if abs(cmd - meas) > max_error:
                raise RuntimeError(
                    f"joint{j + 1} error {abs(cmd - meas):.3f} rad exceeds "
                    f"--max-error {max_error}; aborting for safety"
                )


def main() -> None:
    args = _parse_args()
    poses = _load_poses(args.poses)

    print(f"side={args.side} kp={args.kp} poses={len(poses)} "
          f"approach={args.approach}")
    for i, p in enumerate(poses):
        print(f"  P{i}: {p}")
    if not args.yes:
        reply = input(
            "Arm will move through these poses with gravity comp OFF at "
            "the kp above. Teleop publishers stopped? E-stop in reach? "
            "[y/N] "
        )
        if reply.strip().lower() != "y":
            raise SystemExit("aborted")

    rclpy.init()
    node = _DeflectionMeasurer(args.side)
    node.assert_sole_publisher()
    home = node.wait_for_state()
    print(f"start pose: {[round(v, 3) for v in home]}")

    saved_names = list(MEASUREMENT_PARAMS) + ["mit_kp", "mit_kd"]
    saved = node.get_params(saved_names)
    print(f"saved controller params: {saved}")
    forced = dict(MEASUREMENT_PARAMS)
    forced["mit_kp"] = args.kp
    if args.kd is not None:
        forced["mit_kd"] = args.kd
    node.set_params(forced)

    if args.approach == "both":
        approaches = ["above", "below"]
    elif args.approach == "none":
        approaches = ["direct"]
    else:
        approaches = [args.approach]

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    rows: list[dict] = []
    ok = True
    try:
        for idx, pose in enumerate(poses):
            for approach in approaches:
                pre = list(pose)
                if approach == "above":
                    pre[1] += args.approach_offset
                    pre[2] += args.approach_offset
                elif approach == "below":
                    pre[1] -= args.approach_offset
                    pre[2] -= args.approach_offset
                print(f"P{idx} [{approach}] -> pre-pose")
                node.ramp_to(pre, args.rate, args.max_speed)
                node.hold(pre, args.rate, 1.5, args.max_error)
                node.ramp_to(pose, args.rate, args.max_speed)
                for attempt in (1, 2):
                    mean, std, eff, n = node.hold_and_sample(
                        pose, args.rate, args.settle, args.seconds,
                        args.max_error,
                    )
                    if max(std) <= args.max_std:
                        break
                    print(f"  retry: std {max(std):.4f} > {args.max_std}")
                sag = [c - m for c, m in zip(pose, mean)]
                print(
                    f"  sag(rad) J2={sag[1]:+.4f} J3={sag[2]:+.4f} "
                    f"J4={sag[3]:+.4f}  max_std={max(std):.4f} n={n}"
                )
                row = {
                    "side": args.side, "kp": args.kp, "pose_idx": idx,
                    "approach": approach, "n_samples": n,
                }
                for j in range(6):
                    row[f"q_cmd{j + 1}"] = round(pose[j], 6)
                    row[f"q{j + 1}"] = round(mean[j], 6)
                    row[f"std{j + 1}"] = round(std[j], 6)
                    row[f"eff{j + 1}"] = round(eff[j], 6)
                rows.append(row)
    except (RuntimeError, KeyboardInterrupt) as exc:
        ok = False
        print(f"\nstopping: {exc}")
    finally:
        try:
            print("returning to start pose...")
            node.ramp_to(home, args.rate, args.max_speed)
        finally:
            print(f"restoring controller params: {saved}")
            # Restore mit_kp/mit_kd BEFORE re-enabling compensation: params
            # are applied one at a time, and the controller rejects
            # enabling gravity comp while an uncalibrated (just-measured)
            # kp is active.
            order = sorted(
                saved, key=lambda k: 0 if k.startswith("mit_k") else 1
            )
            node.set_params(
                {k: saved[k] for k in order if saved[k] is not None}
            )
            if rows:
                with open(args.out, "w", newline="") as fh:
                    writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
                    writer.writeheader()
                    writer.writerows(rows)
                print(f"wrote {len(rows)} rows -> {args.out}")
            node.destroy_node()
            rclpy.shutdown()
    if not ok:
        sys.exit(1)
    print(
        "next: python3 gravity_id/fit_deflection.py "
        f"--csv {args.out} --side {args.side}"
    )


if __name__ == "__main__":
    main()
