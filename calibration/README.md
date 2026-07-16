# calibration/

Calibration, controller characterization, and diagnostics for the Piper MIT
compensation stack. This directory is entirely new on this branch — it
supports the compensation features added in `robo_orchard_piper_ros2`. The store schema and value-flow contract live in
[CALIBRATION.md](CALIBRATION.md); this README covers why the tools exist and
how to run them.

## Motivation

The follower controllers compensate gravity by adding a position offset to the
MIT command (see `ros2_package/robo_orchard_piper_ros2/README.md` for why the
torque channel can't be used). That conversion needs the *measured*
torque→deflection behavior of each joint:

- Piper firmware holds position with a stiffness that is **not** the sent kp
  and is **nonlinear in load**, so a scalar constant is not enough — hence
  per-joint deflection tables.
- The behavior **changes with the sent kp**, so calibrations are indexed by kp
  and the controllers refuse to run gravity compensation at a kp that has no
  entry. Changing the deployment kp means re-running the calibration, not
  editing a config.
- Everything measured lands in one store, `deflection_calibrations.json`,
  which launch files and the app read but never contain copies of. That keeps
  episode metadata, launch defaults, and running controllers from drifting
  apart.

## Prerequisites (hardware tools)

Run inside the holobrain container with ROS sourced, the teleop stack up, and
control mode **Stop**. Keep the e-stop in reach. The tools:

- stop the side's takeover muxer and respawn it on exit (`takeover_muxer.py`);
- refuse to start if anything else publishes on `/robot/{side}/joint_cmd` —
  duplicate command publishers caused the kd-buzz incident;
- save and restore all controller parameters on every exit path (kp/kd first,
  so restoring never trips the uncalibrated-kp rejection);
- abort if holding error exceeds 0.35 rad, ramp at ≤ 0.15 rad/s, and return
  the arm to its start pose even on abort. Data collected before an abort is
  kept.

## Tools

### `calibrate_kp.py` — calibrate a deployment kp (~10 min, moves the arm)

```bash
python3 /opt/roboorchard/calibration/calibrate_kp.py --side left --kp 30
```

One-shot wrapper: stops the muxer, runs `measure_deflection.py`, fits with
`fit_deflection.py --update-calibration`, respawns the muxer. This is what the
app's "kp Calibration" panel runs. Re-run per side whenever you want to deploy
a new kp. Flags: `--store`, `--out-dir` (default `/data/holobrain/deflection`),
`--urdf`, `--keep-muxer-stopped`.

### `measure_deflection.py` — static sag measurement (moves the arm)

Holds a grid of 8 static poses with compensation off and records commanded vs.
measured joint positions. The default grid spans J2 torques ~1–10 Nm both
signs (J3 ~1–6, J4 ~0.1–1.4) and is table-safe by construction: extreme-reach
poses curl the wrist and every pose is approached from `--approach-offset`
(default 0.10 rad on J2/J3) above *and* below so stiction cancels in the
average while the pre-pose still clears the table under low-kp sag. Custom
grids via `--poses file.txt` (one pose per line, 6 comma-separated radians).
Key flags: `--kp` (required — the kp being calibrated), `--settle 3.0`,
`--seconds 2.0`, `--max-std 0.002` (unsettled windows retry once), `--yes`.

### `fit_deflection.py` — offline fit (no hardware)

```bash
python3 fit_deflection.py --csv left_kp30_*.csv --side left --update-calibration deflection_calibrations.json
```

Per joint (default `--joints 2,3,4`), builds (|torque|, |sag|) knots — torque
is the URDF gravity torque at the *commanded* pose (`urdf_gravity_check.py`,
an implementation independent of the deployed pinocchio RNEA) — and emits
either a scalar `offset_stiffness` or, when stiffness varies with load by more
than `--spread 1.2`, a monotone deflection table. `--update-calibration`
does an atomic load-modify-write of the store (unknown keys survive); without
it, the fit is printed as `ros2 param set` commands for one-off testing.

### `calibrate_friction.py` — Coulomb friction characterization (~2–4 min, moves the arm)

```bash
python3 /opt/roboorchard/calibration/calibrate_friction.py --side left
```

Constant-speed triangle sweeps on J2/J3 at several speeds; the up-vs-down
deflection gap at matched commanded position (cancels gravity) extrapolated to
speed→0 (cancels viscous/latency terms) gives the Coulomb deflection, which
the store's stiffness at the current kp converts to torque.
**Characterization only**: writes a raw CSV and a `*_fit.json` marked
`characterization_only` under `/data/holobrain/friction`; it reads the store
for stiffness but never writes it and refuses `--result-json` pointing at it.
Known physics: J2 saturates near 0.55 Nm — the residual reversal error is
gear backlash, which no feedforward can remove. Fit math is pure python in
`friction_fit.py` (`pytest calibration/tests/test_friction_fit.py`).

### `urdf_gravity_check.py` — URDF sanity check (no hardware, no ROS)

Dependency-light (numpy + stdlib XML) static gravity model: per-joint gravity
torques at named poses plus an FK report of subtree masses, COM offsets, and
levers. Use it to eyeball a URDF before trusting fits based on it:
`python3 calibration/urdf_gravity_check.py`.

## Support modules

- `store.py` — atomic, permission-preserving store I/O (the store is
  bind-mounted into the container and touched by different users).
- `paths.py` — `DEFAULT_URDF`, read from `configs/piper_urdf_path.txt` so the
  URDF location is configuration, not code.
- `takeover_muxer.py` — find/stop/respawn the side's takeover muxer; skips
  respawn (with a warning to re-run `teleop/dagger.sh`) if the muxer's params
  file vanished with its container `/tmp`.

Tests: `pytest calibration/tests/` (fit math, store I/O) — no hardware needed.

Scripted hardware evaluations (circle tracking, stress runs) are **not** here;
they live in `robo_orchard_teleop_ros2.robot_eval`.
