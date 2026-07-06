# MIT offset-stiffness / deflection-table calibration

Reproducible procedure for generating the two follower-controller
parameters that drive kp-offset gravity compensation:

    mit_gravity_compensation_offset_stiffness   # per-joint Nm/rad
    mit_gravity_compensation_deflection_table   # JSON knots for load-
                                                # dependent joints (J2)

Background: Piper MIT mode is not a kp-spring. The firmware holds the
commanded position with its own internal stiffness `k_eff` (hundreds of
Nm/rad, a function of the sent mit_kp AND of load on J2), so converting
compensation torque to a position offset via `tau / mit_kp` overcompensates
by an order of magnitude. The controller instead uses
`offset = D(tau_rnea(target))` where `D` is the measured
torque-to-deflection curve. See `kp_offset_consistency_check.md` for the
diagnostic history and `ros_bridge.py::_split_gravity_torque` for the
runtime implementation (deflection table > scalar stiffness > kp).

**The measured stiffness depends on mit_kp. Re-run this whole procedure
whenever the deployment kp changes, and fit each kp separately.**

One-command wrapper: `calibrate_kp.py --side left --kp 30` runs the
whole thing (stops/respawns the side's take_over muxer, measures at the
requested kp, fits, writes the calibration store). The inference app
exposes the same flow as the **kp Calibration** button (control mode
must be Stop). The manual steps below are the same procedure unrolled.

## 1. Prepare the stack

1. Launch HoloBrain normally so the `single_ctrl` controllers are up.
2. Stop every other publisher of `/robot/<side>/joint_cmd`: the
   take_over muxer, aloha_orchestrator, and any inference app/policy.
   The measurement script hard-refuses to run while it sees another
   publisher (duplicate publishers caused the kd-buzz incident). E.g.:

       docker exec -it holobrain tmux kill-window -t <robot-control-window>
       # then relaunch only the single_ctrl controllers, or kill the
       # take_over/orchestrator panes individually.

3. Keep the e-stop in reach: the grid includes extended poses with
   ~10 Nm on J2.

## 2. Measure (online, ~10 min per arm)

Inside the container (ROS sourced):

    docker exec -it holobrain bash
    source /opt/ros/humble/setup.bash
    python3 /opt/roboorchard/gravity_id/measure_deflection.py \
        --side left --kp 40 \
        --out /data/holobrain/deflection/left_kp40_$(date +%Y%m%d).csv

- `--side right` for the right arm (same default pose grid; torques are
  identical for equal joint angles on the dual-arm URDF).
- `--kp <value>` must be the kp the arm will actually run at.
- The script disables gravity + friction compensation, forces mit_kp,
  visits each pose from above AND below (stiction cancels in the fit),
  ramps at ≤0.15 rad/s, aborts if any joint deviates >0.35 rad, then
  restores all params and returns the arm to its start pose.
- Custom pose grid: `--poses <file>` with one `q1,...,q6` line per pose.
  Preview torques for a candidate pose first with
  `urdf_gravity_check.gravity_torques()`; aim to span 2–10 Nm on J2.

## 3. Fit (offline, seconds)

    python3 /opt/roboorchard/gravity_id/fit_deflection.py \
        --csv /data/holobrain/deflection/left_kp40_*.csv --side left

Prints per-pose torque/sag/secant diagnostics, warns on sign anomalies
and stiction-dominated knots, then emits the two `ros2 param set` lines
ready to paste. Joints whose secant stiffness varies more than 1.2x
across the load range (J2) get a deflection table; the rest (J3, J4) get
a constant stiffness from a least-squares fit through the origin.

Notes on the fit:
- Torque is RNEA at the **commanded** pose — the same evaluation point
  the controller uses at runtime, so model error cancels to first order.
- The scalar printed for a table-joint is only the fallback used when
  `mit_gravity_compensation_use_deflection_table:=false`; it is fit to
  the low-load secants (the 2026-07-02 left-arm 207 came from the same
  convention).

## 4. Apply and verify

Persist by writing the fit into the kp-indexed calibration store
(`gravity_id/deflection_calibrations.json`, git-tracked):

    python3 /opt/roboorchard/gravity_id/fit_deflection.py \
        --csv /data/holobrain/deflection/left_kp25_*.csv --side left \
        --update-calibration /opt/roboorchard/gravity_id/deflection_calibrations.json

The follower controllers read this store via
`mit_gravity_compensation_calibration_file` (a launch default in
`piper_dagger_compat.launch.py`): at startup and on every `mit_kp`
change they load the `side -> kp` entry matching their kp, and REJECT a
kp with no entry, listing the calibrated values. Switching between
calibrated kps is therefore just:

    ros2 param set /robot/left/robot_left_controller mit_kp 40.0

with no other parameters to touch — and after adding a new kp entry with
`--update-calibration`, that kp becomes accepted immediately (the store
is re-read on each kp change; a restart is only needed to pick up a
changed entry for the kp currently running). The emitted
`ros2 param set offset_stiffness/deflection_table` lines still work as a
manual override for live A/B experiments; they bypass the store until
the next kp change reloads it.

The boot kp lives in two places that must name a CALIBRATED kp:
`follower_mit_kp` in the nexus template
`~/.robot-lab-nexus/templates/teleop/dagger.sh` (boot; note the
container runs the GENERATED copy under
`/opt/robot-lab-nexus/generated/`, regenerated from the template only on
a full `robot-lab-nexus launch`) and `default_follower_kp` in the
inference app's `config.py` (UI seed). If boot names an uncalibrated kp
the controller logs an error and starts with gravity compensation
DISABLED (teleop still works, the follower just sags).

Quick verification: re-run `measure_deflection.py` at the same kp but
with compensation left ON (`ros2 param set ... mit_gravity_compensation_enabled true`
manually after the script disables it — or simply command one of the grid
poses and sample with `sample_joint_mean.py`); settled error on J2–J4
should collapse to within the stiction band (~±0.002 rad). Full
verification: record a teleop episode and run
`~/Hobot/tools/teleop/teleop_analysis/analyze_episode_ee_gravity.py` — the
2026-07-02 left-arm calibration gave 1.19 mm mean / 2.8 mm p95 EE error.

## Provenance

The deployed left-arm values (stiffness `[0, 207, 418, 93, 0, 0]`, J2
knots `(2.61,0.0137) (4.76,0.0213) (7.19,0.0239) (9.93,0.0272)`, measured
at kp=40 on 2026-07-02) were produced manually with `sample_joint_mean.py`
per `kp_offset_consistency_check.md`; the J2 knot torques correspond to
RNEA at commanded poses `[0,1.0,-1.0]`, `[0,~1.27,-1.0]`, `[0,1.57,-1.4]`,
`[0,2.0,-1.4]`. These scripts automate exactly that procedure.
