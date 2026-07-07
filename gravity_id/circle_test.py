#!/usr/bin/env python3
"""Scripted end-effector circle test for one Piper follower arm.

Traces a vertical circle (default 15 cm diameter, 4 s per lap) with the
gripper base and logs commanded vs measured joints so Cartesian tracking
can be scored. Gravity/friction compensation and controller gains are
left at their deployment values -- this is a motion quality test, not a
parameter probe.

Three stages, because pinocchio and rclpy live in different interpreters
inside the holobrain container:

  plan     (needs pinocchio -> /opt/robot-venv/bin/python3)
           FK the seed pose, sweep the circle, IK every waypoint with
           damped least squares, validate limits/velocity/clearance and
           write a joint trajectory JSON.
  run      (needs rclpy -> ROS python3)
           Stream the trajectory to /robot/<side>/joint_cmd at the
           planned rate and record /puppet/joint_<side> to CSV.
  analyze  (needs pinocchio -> /opt/robot-venv/bin/python3)
           FK the logged joints and report radius / diameter / RMS
           Cartesian tracking error.

Typical session INSIDE the holobrain container:

    /opt/robot-venv/bin/python3 /opt/roboorchard/gravity_id/circle_test.py \
        plan --side left
    source /opt/ros/humble/setup.bash
    python3 /opt/roboorchard/gravity_id/circle_test.py run --side left
    /opt/robot-venv/bin/python3 /opt/roboorchard/gravity_id/circle_test.py \
        analyze --side left

PREREQUISITES for `run`:
  * single_ctrl controllers running; control mode Stop (no teleop /
    inference driving the arms). The side's take_over muxer is stopped
    for the run and respawned on exit, and the script refuses to start
    if any other publisher remains on /robot/<side>/joint_cmd.
  * E-stop within reach. A 0.35 rad joint tracking-error guard aborts.

The circle starts AT the seed pose's end-effector point and its center
sits one radius below it (vertical planes), so the arm never goes above
the seed pose and no Cartesian lead-in is needed. Angular speed blends
in/out with a cosine ramp so the trajectory starts and ends at rest.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
URDF_DEFAULT = "/data/holobrain/urdf/piper_x_description_dualarm_v2.urdf"
OUT_DIR_DEFAULT = "/data/holobrain/circle_test"

# Compact, tip-high pose from the validated measure_deflection grid.
SEED_POSE = [0.0, 0.6, -1.4, 0.0, 0.0, 0.0]

JOINT_LIMIT_MARGIN = 0.05  # rad kept clear of URDF limits
MAX_JOINT_SPEED = 1.5      # rad/s planning bound (well under Piper limits)
IK_TOL = 1e-4              # m convergence per waypoint
IK_DAMPING = 1e-3
IK_NULLSPACE_GAIN = 0.05
MIN_LAP_PERIOD = 2.0       # s; 15 cm at 2 s/lap is already ~0.24 m/s


def _traj_path(out_dir: str, side: str) -> str:
    return os.path.join(out_dir, f"circle_traj_{side}.json")


# --------------------------------------------------------------------------
# plan
# --------------------------------------------------------------------------

def _load_reduced_model(urdf: str, side: str):
    import numpy as np
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf)
    keep = [f"{side}_joint{i}" for i in range(1, 7)]
    lock_ids = [
        model.getJointId(model.names[j])
        for j in range(1, model.njoints)
        if model.names[j] not in keep
    ]
    reduced = pin.buildReducedModel(model, lock_ids, pin.neutral(model))
    order = [reduced.names[j] for j in range(1, reduced.njoints)]
    if order != keep:
        raise SystemExit(f"unexpected joint order in reduced model: {order}")
    return reduced, reduced.createData(), np


def _theta_profile(laps: int, period: float, ramp: float, rate: float):
    """Angle-vs-time samples: cosine-blended lead-in/out, cruise at omega.

    Total swept angle is exactly laps*2*pi so the path closes at the top.
    """
    omega = 2.0 * math.pi / period
    total = laps * 2.0 * math.pi
    cruise = total - omega * ramp  # each ramp covers omega*ramp/2
    if cruise < 0.0:
        raise SystemExit("--ramp too long for the requested laps/period")
    duration = laps * period + ramp
    dt = 1.0 / rate
    ts, thetas = [], []
    t, theta = 0.0, 0.0
    while t < duration - 0.5 * dt:
        ts.append(t)
        thetas.append(min(theta, total))
        if t < ramp:
            w = omega * 0.5 * (1.0 - math.cos(math.pi * t / ramp))
        elif t > duration - ramp:
            w = omega * 0.5 * (
                1.0 - math.cos(math.pi * (duration - t) / ramp)
            )
        else:
            w = omega
        theta += w * dt
        t += dt
    ts.append(duration)
    thetas.append(total)
    return ts, thetas


def _plan(args: argparse.Namespace) -> None:
    model, data, np = _load_reduced_model(args.urdf, args.side)
    import pinocchio as pin

    ee_frame = args.ee_frame or f"{args.side}_gripper_base"
    if not model.existFrame(ee_frame):
        raise SystemExit(f"frame {ee_frame!r} not in {args.urdf}")
    fid = model.getFrameId(ee_frame)

    q_seed = np.array(args.seed, dtype=float)
    lo = model.lowerPositionLimit + JOINT_LIMIT_MARGIN
    hi = model.upperPositionLimit - JOINT_LIMIT_MARGIN

    def fk(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation.copy()

    start = fk(q_seed)
    r = args.diameter / 2.0
    axes = {
        "xz": (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
        "yz": (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])),
        "xy": (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])),
    }
    a_hat, n_hat = axes[args.plane]
    if args.reverse:
        a_hat = -a_hat
    center = start - r * n_hat  # circle top = seed EE point

    ts, thetas = _theta_profile(args.laps, args.period, args.ramp, args.rate)

    def target(theta):
        return center + r * (math.sin(theta) * a_hat + math.cos(theta) * n_hat)

    # Damped-least-squares position IK, warm-started along the path, with
    # a nullspace pull toward the seed posture to keep the wrist settled.
    qs, q = [], q_seed.copy()
    worst_resid = 0.0
    for theta in thetas:
        p_des = target(theta)
        for _ in range(60):
            err = p_des - fk(q)
            if float(np.linalg.norm(err)) < IK_TOL:
                break
            jac = pin.computeFrameJacobian(
                model, data, q, fid, pin.LOCAL_WORLD_ALIGNED
            )[:3, :]
            jjt = jac @ jac.T + (IK_DAMPING ** 2) * np.eye(3)
            dq = jac.T @ np.linalg.solve(jjt, err)
            dq += (
                np.eye(6) - np.linalg.pinv(jac) @ jac
            ) @ (IK_NULLSPACE_GAIN * (q_seed - q))
            q = np.clip(q + dq, lo, hi)
        resid = float(np.linalg.norm(p_des - fk(q)))
        worst_resid = max(worst_resid, resid)
        if resid > 10 * IK_TOL:
            raise SystemExit(
                f"IK failed at theta={theta:.2f} rad (residual "
                f"{resid * 1000:.2f} mm) -- circle leaves the reachable "
                "workspace; move the seed pose or shrink --diameter."
            )
        qs.append(q.copy())

    qs = np.array(qs)
    vels = np.gradient(qs, np.array(ts), axis=0)
    peak_speed = float(np.abs(vels).max())
    if peak_speed > MAX_JOINT_SPEED:
        raise SystemExit(
            f"peak joint speed {peak_speed:.2f} rad/s exceeds "
            f"{MAX_JOINT_SPEED}; increase --period."
        )

    ee = np.array([fk(qi) for qi in qs])
    if float(ee[:, 2].min()) < args.min_z:
        raise SystemExit(
            f"path z-min {ee[:, 2].min():.3f} m is below --min-z "
            f"{args.min_z}; raise the seed pose or use --plane xy."
        )

    os.makedirs(args.out_dir, exist_ok=True)
    path = _traj_path(args.out_dir, args.side)
    payload = {
        "side": args.side,
        "urdf": args.urdf,
        "ee_frame": ee_frame,
        "plane": args.plane,
        "diameter": args.diameter,
        "period": args.period,
        "laps": args.laps,
        "ramp": args.ramp,
        "rate": args.rate,
        "seed": list(map(float, q_seed)),
        "center": list(map(float, center)),
        "t": [round(float(v), 5) for v in ts],
        "q": [[round(float(v), 6) for v in row] for row in qs],
        "qd": [[round(float(v), 5) for v in row] for row in vels],
    }
    with open(path, "w") as fh:
        json.dump(payload, fh)

    print(f"seed EE point       : {np.round(start, 4).tolist()} m")
    print(f"circle center       : {np.round(center, 4).tolist()} m")
    print(
        f"path bbox x[{ee[:, 0].min():+.3f},{ee[:, 0].max():+.3f}] "
        f"y[{ee[:, 1].min():+.3f},{ee[:, 1].max():+.3f}] "
        f"z[{ee[:, 2].min():+.3f},{ee[:, 2].max():+.3f}] m"
    )
    print(f"duration            : {ts[-1]:.2f} s ({len(ts)} samples)")
    print(f"worst IK residual   : {worst_resid * 1000:.2f} mm")
    print(f"peak joint speed    : {peak_speed:.2f} rad/s")
    print(
        "joint ranges        : "
        + " ".join(
            f"J{j + 1}[{qs[:, j].min():+.2f},{qs[:, j].max():+.2f}]"
            for j in range(6)
        )
    )
    print(f"trajectory -> {path}")
    print("Check the bbox against the physical rig before `run`.")


# --------------------------------------------------------------------------
# run
# --------------------------------------------------------------------------

def _run(args: argparse.Namespace) -> None:
    import rclpy

    sys.path.insert(0, str(HERE))
    from calibrate_kp import _respawn_muxer, _stop_muxer
    from probe_kd_vdes import _Prober

    path = _traj_path(args.out_dir, args.side)
    try:
        traj = json.loads(Path(path).read_text())
    except OSError as exc:
        raise SystemExit(f"no trajectory at {path} (run `plan` first): {exc}")
    if traj["side"] != args.side:
        raise SystemExit(
            f"trajectory was planned for side {traj['side']!r}"
        )

    ts = traj["t"]
    qs = traj["q"]
    qds = traj["qd"]
    rate = float(traj["rate"])
    print(
        f"circle: d={traj['diameter'] * 100:g} cm, {traj['period']:g} s/lap, "
        f"{traj['laps']} lap(s) + {traj['ramp']:g} s blends, plane "
        f"{traj['plane']}, {ts[-1]:.1f} s total, ee={traj['ee_frame']}"
    )
    if not args.yes:
        reply = input(
            f"The {args.side} follower arm will move to "
            f"{[round(v, 2) for v in traj['seed']]} and trace the circle "
            "above. take_over muxer is stopped for the run. Control mode "
            "Stop? E-stop in reach? Workspace clear? [y/N] "
        )
        if reply.strip().lower() != "y":
            raise SystemExit("aborted")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(args.out_dir, exist_ok=True)
    out_csv = os.path.join(
        args.out_dir, f"circle_run_{args.side}_{stamp}.csv"
    )

    muxer_argv = _stop_muxer(args.side)
    rclpy.init()
    node = _Prober(args.side)
    exit_code = 0
    home = None
    saved = {}
    try:
        node.assert_sole_publisher()
        home = node.wait_for_state()
        print(f"start pose: {[round(v, 3) for v in home]}")

        if args.vff:
            names = [
                "mit_velocity_feedforward",
                "mit_velocity_feedforward_source",
                "mit_velocity_feedforward_deadband",
            ]
            saved = node.get_params(names)
            print(f"saved params: {saved}")
            node.set_params(
                {
                    "mit_velocity_feedforward": True,
                    "mit_velocity_feedforward_source": "message",
                    "mit_velocity_feedforward_deadband": 0.0,
                }
            )

        node.ramp_to(list(qs[0]), rate, args.max_speed)
        node.hold_and_sample(
            list(qs[0]), [0.0] * 6, rate, args.settle, 0.5, args.max_error
        )

        print("tracing circle...")
        rows = []
        idx = 0
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t >= ts[-1]:
                break
            while idx + 1 < len(ts) and ts[idx + 1] <= t:
                idx += 1
            # Linear interpolation between planned samples keeps the
            # command smooth even if the loop misses ticks.
            if idx + 1 < len(ts):
                a = (t - ts[idx]) / (ts[idx + 1] - ts[idx])
                q = [
                    (1 - a) * qs[idx][j] + a * qs[idx + 1][j]
                    for j in range(6)
                ]
                qd = [
                    (1 - a) * qds[idx][j] + a * qds[idx + 1][j]
                    for j in range(6)
                ]
            else:
                q, qd = list(qs[-1]), [0.0] * 6
            node.publish_cmd(q, qd if args.vff else None)
            node._check_error(q, args.max_error)
            if node.latest is not None:
                rows.append(
                    [t]
                    + q
                    + list(node.latest)
                    + list(node.latest_effort or [0.0] * 6)
                )
            node._spin_sleep(1.0 / rate)
        # settle back on the closing pose (= opening pose) briefly
        node.hold_and_sample(
            list(qs[-1]), [0.0] * 6, rate, 1.0, 0.5, args.max_error
        )
        with open(out_csv, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                ["t"]
                + [f"cmd{j + 1}" for j in range(6)]
                + [f"meas{j + 1}" for j in range(6)]
                + [f"eff{j + 1}" for j in range(6)]
            )
            writer.writerows(
                [[f"{v:.6f}" for v in row] for row in rows]
            )
        print(f"{len(rows)} samples -> {out_csv}")
        print(
            "next: /opt/robot-venv/bin/python3 "
            f"{HERE / 'circle_test.py'} analyze --side {args.side}"
        )
    except (RuntimeError, KeyboardInterrupt, SystemExit) as exc:
        exit_code = 1
        print(f"\nstopping: {exc}")
    finally:
        if saved:
            try:
                print(f"restoring params: {saved}")
                node.set_params(
                    {k: v for k, v in saved.items() if v is not None}
                )
            except Exception as exc:
                print(f"WARNING: param restore failed: {exc}")
        try:
            if home is not None and node._pub is not None:
                print("returning to start pose...")
                node.ramp_to(home, rate, args.max_speed)
        except Exception as exc:
            print(f"WARNING: return ramp failed: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        if muxer_argv:
            _respawn_muxer(muxer_argv)
    sys.exit(exit_code)


# --------------------------------------------------------------------------
# analyze
# --------------------------------------------------------------------------

def _analyze(args: argparse.Namespace) -> None:
    model, data, np = _load_reduced_model(args.urdf, args.side)
    import pinocchio as pin

    traj = json.loads(Path(_traj_path(args.out_dir, args.side)).read_text())
    fid = model.getFrameId(traj["ee_frame"])

    if args.csv:
        run_csv = args.csv
    else:
        runs = sorted(
            Path(args.out_dir).glob(f"circle_run_{args.side}_*.csv")
        )
        if not runs:
            raise SystemExit(f"no run CSVs in {args.out_dir}")
        run_csv = str(runs[-1])
    print(f"analyzing {run_csv}")

    rows = list(csv.DictReader(open(run_csv)))
    if len(rows) < 100:
        raise SystemExit(f"only {len(rows)} samples in {run_csv}")

    def fk(q6):
        pin.forwardKinematics(model, data, np.array(q6))
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation.copy()

    cmd = np.array(
        [fk([float(r[f"cmd{j + 1}"]) for j in range(6)]) for r in rows]
    )
    meas = np.array(
        [fk([float(r[f"meas{j + 1}"]) for j in range(6)]) for r in rows]
    )
    center = np.array(traj["center"])
    plane_axes = {"xz": (0, 2), "yz": (1, 2), "xy": (0, 1)}
    i, k = plane_axes[traj["plane"]]
    off_axis = ({0, 1, 2} - {i, k}).pop()

    # Score only the cruise portion (blends are intentionally not on the
    # circle at speed).
    ts = np.array([float(r["t"]) for r in rows])
    ramp = float(traj["ramp"])
    sel = (ts > ramp) & (ts < ts[-1] - ramp)

    def radius(p):
        return np.hypot(p[sel, i] - center[i], p[sel, k] - center[k])

    r_cmd, r_meas = radius(cmd), radius(meas)
    err = np.linalg.norm(meas - cmd, axis=1)[sel]
    wobble = meas[sel, off_axis] - center[off_axis]
    print(f"target diameter     : {traj['diameter'] * 1000:.1f} mm")
    print(
        f"commanded diameter  : {2 * r_cmd.mean() * 1000:.1f} mm "
        f"(sd {2 * r_cmd.std() * 1000:.2f})"
    )
    print(
        f"measured diameter   : {2 * r_meas.mean() * 1000:.1f} mm "
        f"(sd {2 * r_meas.std() * 1000:.2f}, "
        f"min {2 * r_meas.min() * 1000:.1f}, "
        f"max {2 * r_meas.max() * 1000:.1f})"
    )
    print(
        f"tracking error      : rms "
        f"{np.sqrt((err ** 2).mean()) * 1000:.1f} mm, "
        f"max {err.max() * 1000:.1f} mm"
    )
    print(
        f"out-of-plane wobble : rms "
        f"{np.sqrt((wobble ** 2).mean()) * 1000:.1f} mm, "
        f"max {np.abs(wobble).max() * 1000:.1f} mm"
    )

    ee_csv = run_csv.replace(".csv", "_ee.csv")
    with open(ee_csv, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            ["t", "cmd_x", "cmd_y", "cmd_z", "meas_x", "meas_y", "meas_z"]
        )
        for n in range(len(rows)):
            writer.writerow(
                [f"{ts[n]:.4f}"]
                + [f"{v:.5f}" for v in cmd[n]]
                + [f"{v:.5f}" for v in meas[n]]
            )
    print(f"EE paths -> {ee_csv}")


# --------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--side", choices=("left", "right"), required=True)
    common.add_argument("--out-dir", default=OUT_DIR_DEFAULT)
    common.add_argument("--urdf", default=URDF_DEFAULT)

    p = sub.add_parser("plan", parents=[common])
    p.add_argument("--diameter", type=float, default=0.15)
    p.add_argument(
        "--period", type=float, default=4.0, help="seconds per lap"
    )
    p.add_argument("--laps", type=int, default=2)
    p.add_argument(
        "--ramp", type=float, default=1.0,
        help="cosine speed blend at start/end (s)",
    )
    p.add_argument("--rate", type=float, default=200.0)
    p.add_argument(
        "--plane", choices=("xz", "yz", "xy"), default="xz",
        help="circle plane; xz = sagittal (default)",
    )
    p.add_argument("--reverse", action="store_true")
    p.add_argument(
        "--seed", type=float, nargs=6, default=SEED_POSE,
        help="joint pose whose EE point is the top of the circle",
    )
    p.add_argument("--ee-frame", default="")
    p.add_argument(
        "--min-z", type=float, default=0.10,
        help="reject plans whose EE goes below this height (m)",
    )

    r = sub.add_parser("run", parents=[common])
    r.add_argument(
        "--vff", action="store_true",
        help="force message-source velocity feedforward for the run "
        "(restored after); halves tracking lag per the kd probe",
    )
    r.add_argument("--max-speed", type=float, default=0.15)
    r.add_argument("--max-error", type=float, default=0.35)
    r.add_argument("--settle", type=float, default=2.0)
    r.add_argument("--yes", action="store_true")

    a = sub.add_parser("analyze", parents=[common])
    a.add_argument("--csv", default="", help="run CSV (default: newest)")

    args = parser.parse_args()
    if args.cmd == "plan":
        if not 0.02 <= args.diameter <= 0.30:
            raise SystemExit("--diameter outside [0.02, 0.30] m")
        if args.period < MIN_LAP_PERIOD:
            raise SystemExit(f"--period below {MIN_LAP_PERIOD} s")
        if args.laps < 1:
            raise SystemExit("--laps must be >= 1")
        _plan(args)
    elif args.cmd == "run":
        _run(args)
    else:
        _analyze(args)


if __name__ == "__main__":
    main()
