# Piper MIT compensation calibration

All deployed calibration values live in **one runtime store**:
`calibration/deflection_calibrations.json` (mounted in the container at
`/opt/roboorchard/calibration/deflection_calibrations.json`, which is the
`follower_gravity_calibration_file` launch default). The follower
controllers load it directly; nothing calibrated is hardcoded in launch
files or app configs.

Follower friction compensation and inertial torque feedforward are not
production features. Friction collection commands physical hardware, but its
fit output is a standalone characterization artifact under
`/data/holobrain/friction`; it cannot update the runtime store or push live
controller parameters.

## Store layout (store_version 2)

```jsonc
{
  "store_version": 2,

  // kp-indexed deflection calibrations (per side). k_eff depends on the
  // sent mit_kp, so entries are only valid at their own kp; the
  // controller rejects an uncalibrated kp, listing the available ones.
  // Keys under a side MUST stay numeric.
  "left":  { "25": { "offset_stiffness": [...], "deflection_table": {...},
                     "measured": "...", "source": "..." }, "40": { ... } },
  "right": { ... },

  // kp-independent production gravity scale (per side).
  "gravity": {
    "left":  { "scale_per_joint": [1, 1, 1, 1, 1, 1], ... },
    "right": { ... }
  }
}
```

## How values flow

1. `piper_dagger_compat.launch.py` passes the store path + side to each
   follower controller (`mit_gravity_compensation_calibration_file` /
   `..._calibration_side`). It contains **no calibrated numbers**.
2. `single.py` loads at startup: the deflection entry for the current
   `mit_kp` (rejecting uncalibrated kp), then the production `gravity`
   section. Loaded values
   are synced back to the parameter server, so `ros2 param get` shows
   what is in effect.
3. Reload triggers at runtime on `mit_kp` changes and gravity-compensation
   enable or calibration file/side changes.
4. The inference app never pushes coefficients (Apply sends toggles and
   kp/kd only) and reads the same store for its episode-metadata
   snapshot, so metadata cannot drift from what runs.

## Calibration and characterization tools

The online tools stop the side's take_over muxer, refuse to run alongside
another `joint_cmd` publisher (kd-buzz protection), restore all controller
params on exit, and respawn the muxer. Run inside the holobrain container with
ROS sourced and control mode Stop; keep the e-stop in reach.

### kp / deflection — `calibrate_kp.py` (~10 min)

```bash
python3 /opt/roboorchard/calibration/calibrate_kp.py --side left --kp 30
```

Static pose grid with comp off (`measure_deflection.py`), then
`fit_deflection.py --update-calibration` writes the kp entry. Re-run
whenever the deployment kp changes. This is the calibration workflow
exposed by the app UI.

### Friction characterization — `calibrate_friction.py` (~2-4 min)

```bash
python3 /opt/roboorchard/calibration/calibrate_friction.py \
  --side left
```

Sweeps J2/J3 through constant-speed triangles at several speeds with
friction comp off. Per joint: up-vs-down deflection gap at the same
commanded position (cancels gravity sag) is extrapolated to speed → 0
(cancels velocity-feedforward lead / latency / viscous terms); the
surviving Coulomb deflection times the store's `offset_stiffness` at
the current kp gives the torque. It reads the deflection store only for that
stiffness conversion and writes a raw CSV plus a
`*_friction_*_fit.json` result marked `characterization_only`. It never
changes the runtime store or controller parameters. Useful flags include
`--joints`, `--speeds`, `--min-r2`, and `--result-json`.
Fit math is in `friction_fit.py` (pure python,
`pytest calibration/tests/test_friction_fit.py`).

Known physics: J2 saturates near 0.55 Nm — the residual reversal error
is gear backlash, which no feedforward torque can remove; pushing past
it risks reversal overshoot. At ~0.5 Nm against 9-12 Nm gravity
torques, friction comp's EE effect is small (mm-scale at reversals).

### Gravity per-joint scale

No dedicated tool yet; the `gravity` section is seeded neutral
(`[1]*6`). Edit by hand from a shape-check (e.g. phase1/known-mass
procedure) — the controllers reload it on the next gravity-comp enable
or restart.

## UI

The app's MIT panel has a kp Calibration panel and a read-only Calibration
store expander. Friction characterization is not exposed as a deployable app
control.

## Store consumers (update together when the schema changes)

- `robo_orchard_piper_ros2/ros_bridge.py` — `resolve_deflection_calibration`
  and `resolve_gravity_scale_calibration`
  (tests: `tests/test_deflection_calibration.py`)
- `robo_orchard_inference_app/components/main_control.py` —
  `_load_calibration_store`, `_calibrated_kps`
- `calibration/fit_deflection.py` — load-modify-write updates for deflection
  entries (unknown keys survive)

Scripted motion evaluations live under
`robo_orchard_teleop_ros2.robot_eval`.
