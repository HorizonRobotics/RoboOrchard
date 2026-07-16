# Scripted masters

New on this branch. Deterministic command publishers that stand in for the human master during
hardware evaluation and data collection. Both publish 7-joint
`sensor_msgs/JointState` (joints 1–6 + gripper) at 100 Hz to
`/left_algo_cmd` / `/right_algo_cmd` and read follower state from
`/puppet/joint_{side}`, so they slot into the takeover-muxer stack in Auto
mode without touching controller parameters.

## Motivation

Teleop recordings are unrepeatable, so controller changes (gravity
compensation, velocity feedforward, kp/kd) could not be A/B compared — and
back-to-back comparison matters because thermal drift alone moves tracking
error by ~0.8 mm over two hours. These nodes make the commanded trajectory a
controlled variable. Safety follows from determinism: every trajectory either
comes from a reviewed scenario JSON (`robot_eval/scenarios/`), a
speed-validated plan, or a previously demonstrated safe path, and every run
ramps in from the arm's actual pose instead of jumping.

## `scripted_joint_master` (joint_master.py)

Three modes, selected by the `mode` parameter or (preferably) a `scenario`
name resolved through the robot_eval registry:

- **sine** — multi-frequency per-joint sweep. The trajectory center is shifted
  so the value at t=0 equals the current measured position (no start jump).
- **waypoints** — lockstepped min-jerk segments through demonstrated dual-arm
  waypoint lists, with per-waypoint settle dwell, `pass_through_indices` for
  poses that must not be dwelled at (thermal limits), and `laps`
  (forward+reverse passes). Segment timing derives from `peak_velocity` /
  `max_accel` × `speed_scale`.
- **circle** — playback of an IK-planned end-effector circle
  (`robot-eval plan circle`); `speed_scale` is clamped to ≤ 1.0 because plans
  are validated at planned speed — only slowing down is safe.

Waypoint and circle runs prepend a slow ramp-in (0.25 rad/s, 1.0 rad/s² caps,
independent of `speed_scale`) from the current pose to the trajectory start.

Useful parameters (defaults): `rate_hz=100`, `duration_s=10` (0 = play plan to
completion), `start_delay_s=1.0`, `use_current_state=true`,
`publish_left/right=true`, `mirror_right=false`, `result_path=""` (set to log
the common robot_eval CSV schema — line-buffered, partial runs stay readable).

Coordination hooks for recorded runs: the node touches `ready_file` once set
up, then waits for `start_trigger_file` to exist before moving, and can wait
for `min_command_subscribers` on its command topics. The inference app uses
these to start the recorder between "ready" and "go", so episode length tracks
the requested duration instead of process-startup jitter.

Run directly:

```bash
python3 joint_master.py --ros-args -p scenario:=stress -p speed_scale:=0.3 \
  -p result_path:=/data/holobrain/robot_eval/stress_manual.csv
```

or through the CLI/UI: `robot-eval run stress --speed-scale 0.3`, or the app's
Scripted Motion panel.

## `recorded_replay_master` (recorded_replay_master.py)

Replays a *measured* safe dual-arm path for gravity/system-ID data collection.
Input is a path-parameterized CSV (`path_s, t_demo_s, left_joint1..gripper,
right_joint1..gripper`) exported from a recorded episode; replay speed is set
in path-seconds per wall second (`replay_speed_path_s_per_s=0.25`) or with the
original demo timing (`use_demo_timing`), optionally `reverse`,
`bidirectional`, or `loop`.

Phases: optional controller reset (`reset_before_start` drives the arms'
`reset_ctrl` services to a configured pose) → `approach_start` interpolation
from the current pose → `hold_start` → move → optional turnaround/reverse →
`hold_end`. Abort guards: stale follower state (`state_timeout_s=0.25`) and
tracking error (`tracking_error_stop_rad=0.25`, warn at 0.12).

Output CSV logs per-side commanded/measured positions and velocities plus a
`tau_label` column (`kp·(cmd−meas) + kd·(v_cmd−v_meas) + torque_ref` with the
configured kp/kd) — the regression target for gravity-model fitting.

```bash
ros2 launch robo_orchard_teleop_ros2 recorded_replay_master.launch.py \
  reference_csv:=/data/holobrain/gravity_id/safe_path.csv \
  output_csv:=/data/holobrain/gravity_id/replay_001.csv \
  replay_speed_path_s_per_s:=0.15 bidirectional:=true
```

## Which one do I want?

- Repeatable controller evaluation with pass/fail analysis → a
  `robot_eval` scenario via `scripted_joint_master` (add new trajectories as
  scenario JSONs, not code — see `../robot_eval/README.md`).
- Slow, guarded re-execution of a demonstrated path to harvest torque/tracking
  data → `recorded_replay_master`.
