# kp-offset consistency checks (no calibration, no fitted parameters)

Purpose: decide *why* the follower floats above the master in kp-offset-only
gravity-comp mode. Each check below tests an identity that must hold exactly if
the pipeline math is lawful. Nothing is fitted; every number is either
commanded, measured, or read from the recorder CSV.

Model of the static equilibrium (per joint, everything signed):

    comp OFF:  kp_fw * (q_cmd - q_off) = tau_true
    comp ON:   kp_fw * (q_cmd + o - q_on) = tau_true
    =>  q_on = q_off + o                                   ... Identity (1)

where `o` is the commanded position offset (CSV column `kp_position_offset_j`)
and `kp_fw` is whatever stiffness the firmware actually applies. Identity (1)
contains no model torque and no kp value — it must hold no matter how wrong
the URDF or the firmware gain are, as long as the kp channel is linear.

## Setup (once)

1. Rebuild `robo_orchard_piper_ros2` in the container and restart the stack so
   `mit_gravity_compensation_use_t_ref` exists.
2. Kill the teleop command publishers (take_over + aloha_orchestrator) but keep
   the `single_ctrl` controllers running. Then verify there is exactly ONE
   publisher before publishing commands yourself — duplicate `joint_cmd`
   publishers caused the kd-buzz incident:

       ros2 topic info /robot/left/joint_cmd   # Publisher count must be 1
                                               # (your `topic pub` below)

3. Pick 3–5 static poses with meaningful gravity torque on J2/J3. Known-safe
   starting points (verified for the LEFT arm in phase1_shape_check.py; always
   preview before use, and keep the e-stop in reach):

       P1 = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]
       P2 = [0.0, 0.6, -1.4, 0.0, 0.0, 0.0]
       P3 = [0.0, 1.4, -1.0, 0.0, 0.0, 0.0]   # preview first

4. Controller params for the test (left arm shown; node
   `/robot/left/robot_left_controller`, measured topic `/puppet/joint_left`):

       ros2 param set $N mit_gravity_compensation_use_t_ref false
       ros2 param set $N mit_gravity_compensation_use_kp_offset true
       ros2 param set $N mit_friction_compensation_enabled false
       ros2 param set $N mit_gravity_compensation_record_path /tmp/gc_left.csv

5. Command a fixed pose (7th value = gripper; 0.0 closes it):

       ros2 topic pub -r 50 /robot/left/joint_cmd sensor_msgs/msg/JointState \
           "{position: [0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]}"

6. Measure settled positions with the sampler (2 s window after 3 s settle):

       python3 gravity_id/sample_joint_mean.py --topic /puppet/joint_left \
           --settle 3 --seconds 2

## Check 0 — stiction band (sets the tolerance for everything else)

At one pose, comp OFF: approach the pose from above (publish pose with
J2 +0.15 rad for ~3 s, then the pose) and record `q_hi`; approach from below
(J2 −0.15 rad first) and record `q_lo`. The band `|q_hi - q_lo|` per joint is
the stiction dead zone width; call it `b_j`. All later comparisons are only
meaningful to within `±b_j/2`. Approach every later measurement from the SAME
direction (e.g. always from below) so stiction bias cancels in differences.

## Check 1 — pipeline identity (firmware kp channel is lawful)

Same commanded pose throughout, same approach direction:

1. `mit_gravity_compensation_enabled false` → sample → `q_off`.
2. `mit_gravity_compensation_enabled true`  → sample → `q_on`; average
   `kp_position_offset_j` over the same window from the CSV → `o_j`
   (with `use_t_ref false`, also confirm `residual_tau_j ≈ 0` and
   `applied_tau_j = 0` in the CSV).
3. Verdict per joint:  `q_on_j - q_off_j - o_j` should be 0 within `±b_j/2`.

PASS → software + firmware kp channel behave exactly as the math says; the
float is entirely `o_j - (q_cmd_j - q_off_j)`, i.e. model-vs-reality, and
proceed to Checks 2/3.
FAIL → the firmware kp channel itself is nonlinear/scaled per-state; stop
here, that is the bug, and no scale can fix it.

## Check 2 — firmware kp is proportional to the kp we send

Comp OFF, same pose, same approach direction. Sample sag at `mit_kp 10`
(`s10_j = q_cmd_j - q_off_j`) and at `mit_kp 15` (`s15_j`). Increase kp rather
than decrease it — lowering kp grows the sag and the arm swings low (collision
risk).

    s15_j / s10_j must equal 10/15 = 0.667 (within stiction tolerance)

PASS → `kp_fw = alpha * kp_sent` with constant alpha. FAIL → firmware kp
mapping is not proportional; measure a few more kp values to map it.

## Check 3 — one unknown constant vs. wrong model shape

For each pose P and joint j with meaningful torque (|desired_tau_j| > 1 Nm):

    r_j(P) = o_j(P) / (q_cmd_j(P) - q_off_j(P))     # offset / sag, signed

By the equilibrium equations, `r_j(P) = alpha * tau_model / tau_true` at that
pose.

- `r_j` the SAME constant for all poses and joints → behavior matches the math
  up to a single global gain. A uniform firmware stiffness unit and a uniform
  model mass error are then mathematically indistinguishable from position
  data (identifiability limit); dividing the compensation by that constant is
  exact, not a fudge. Separating alpha from the model absolutely requires one
  known external torque (e.g. the 530 g bottle at a known radius).
- `r_j` constant per joint but different between joints → per-joint scale error
  (masses), still exactly correctable via `scale_per_joint / r_j`.
- `r_j` varies WITH POSE for a fixed joint → the model torque *function* has
  the wrong shape (COM direction); no scale fixes it, re-identify first
  moments (phase1 tool, shape stage).

## Bookkeeping

Record for every measurement: pose, approach direction, comp state, mit_kp,
sampler JSON line, and the CSV window (t_unix range). The CSV columns needed:
`kp_position_offset*`, `kp_offset_tau*`, `desired_tau*`, `applied_tau*`,
`residual_tau*`.

> Reproducing these values is now automated: see
> `DEFLECTION_CALIBRATION.md` (`measure_deflection.py` +
> `fit_deflection.py`). The manual procedure below is kept for the
> diagnostic rationale.

## Outcome (2026-07-02, left arm) — for the record

These checks led to the final findings; see the deflection-table support in
`ros_bridge.py`/`single.py` for the resulting implementation:

- Firmware MIT mode is NOT a kp-spring: static stiffness ~100-400 Nm/rad,
  with `k ≈ a + b·kp` (kp is an input, not the stiffness) and load-dependent
  (progressive) on J2.
- Correct compensation: `p_ref = target − D(τ_rnea(target))`, `t_ff = 0`,
  where D is the measured deflection curve at the operating kp.
- Measured at kp=40: J2 knots (2.61, 0.0137), (4.76, 0.0213), (7.19, 0.0239),
  (9.93, 0.0272); J3 ≈ 418 Nm/rad; J4 ≈ 93 Nm/rad. Result: left EE error
  1.19 mm mean / 2.8 mm p95 (episode 2026_07_02-11_58_20).
