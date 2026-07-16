# robo_orchard_piper_ros2

This package includes the ros2 control node of the piper arm.

Adapt from the original [piper_ros](https://github.com/agilexrobotics/piper_ros)

---

## Added on this branch: MIT compensation stack

Upstream, `single_ctrl` drives the arm in position mode or a bare fixed-gain
MIT toggle. This branch adds a hardware-calibrated compensation stack on the
MIT path:

- runtime-tunable `mit_kp` / `mit_kd` / `mit_vel_ref` / `mit_torque_ref`
- gravity compensation delivered through calibrated position offsets
- velocity feedforward with phase lead
- an oscillation guard
- a kp-indexed calibration store the controllers load and enforce
- teach-mode-aware `enable_ctrl` and a `reset_ctrl` that re-seats the MIT
  setpoint so re-entering MIT does not jump

Code: `ros_bridge.py` (gravity compensator, calibration resolvers,
torque→offset split), `single.py` (parameters, pipeline, services),
`oscillation_guard.py` (new).

### Why position offsets, not torque

Hardware measurement showed Piper MIT mode is not the impedance controller it
appears to be:

1. **Firmware holds position with its own finite stiffness.** Joints sag
   under gravity by a nonlinear, torque-dependent deflection; raising kp
   doesn't fix it and invites chatter.
2. **The `t_ref` channel is weak** (8-bit over ±8 Nm, floors small values),
   while J2 gravity torque alone reaches ~10–12 Nm at full reach.
3. **Sent kd is applied ~10× by firmware** and damps absolute velocity —
   it drags against commanded motion (~30–50 ms lag) unless `v_des` is fed.

So: RNEA gravity torque is converted to a position offset through a measured
per-joint, per-kp torque→deflection table; `v_des` comes from filtered
command velocity with a phase lead canceling the filter delay (halved teleop
lag, hardware-validated); and the guard ramps out feedback-sensitive terms if
the command stream rings at 1.25–7 Hz. Production config
(`piper_dagger_compat.launch.py`): kp 25 / kd 0.8, all compensation via the
position offset (`use_t_ref=false`), phase lead 1.0, friction off.

### Calibration store

Controllers load `calibration/deflection_calibrations.json` (launch default
`/opt/roboorchard/calibration/deflection_calibrations.json`) at startup and on
`mit_kp` / enable / file / side changes. Deflection entries are kp-indexed:
setting an uncalibrated `mit_kp` with gravity compensation enabled is rejected
at runtime. Calibrate a new kp with `calibration/calibrate_kp.py`. Resolved
values are synced back to the parameter server. See
`calibration/CALIBRATION.md` (schema) and `calibration/README.md` (tools).

### Parameters

All runtime-tunable with validation.

| Group | Parameters (defaults) |
|---|---|
| MIT gains | `mit_kp` (10.0), `mit_kd` (0.8), `mit_vel_ref` (45.0), `mit_torque_ref` (0.0) |
| Velocity ff | `mit_velocity_feedforward` (false), `_source` (`position_delta`), `_cutoff_hz` (10.0), `_deadband` (0.02), `_scale` ([1]×6), `_phase_lead` (1.0) |
| Accel filter | `mit_command_acceleration_filter_cutoff_hz` (8.0), `_alpha` (0.15) |
| Gravity comp | `mit_gravity_compensation_enabled` (false), `_urdf_path`, `_scale` / `_scale_per_joint`, `_use_t_ref` (true; false in production), `_max_t_ref` (8.0), `_use_kp_offset` (true), `_offset_stiffness`, `_deflection_table` / `_use_deflection_table` (true), `_calibration_file` / `_calibration_side`, `_record_path` |
| Friction (characterization only) | `mit_friction_compensation_enabled` (false), `_scale`, `_load_scale`, `_min/full/taper_velocity`, `_static_scale`, `_static_velocity` |
| Oscillation guard | `mit_comp_oscillation_guard` (true), `_min_amplitude` (0.15 rad/s), `_trip_half_cycles` (5) |

### Diagnostics

`mit_gravity_compensation_record_path` streams a per-step CSV (commanded q,
RNEA torque, applied torque, position offset, residual, guard gain). The
inference app routes these into recorded episode directories.

### Safety notes

- Never run two publishers on one `joint_cmd` topic — duplicate publishers
  produce a violent kd buzz. The calibration tools refuse to start if the
  topic already has a publisher.
- Keep kd ≤ ~1.2 in experiments.
- Don't work around the uncalibrated-kp rejection; recalibrate instead.
