# RoboOrchard Inference App

Including visualization, inference, dagger and data recording.

---

## Added on this branch

The upstream app already provides the Streamlit console: Foxglove
visualization, episode configuration, the data recorder, control-mode
switching (Takeover/Auto/Stop), arm enable/disable/reset, inference control,
hand-eye calibration, and task configuration. This branch adds MIT tuning,
calibration, scripted motion, and hardening around recording and control
transitions.

### New panels and features

- **MIT Params + Apply** (`components/mit_control.py`, new): master/follower
  MIT mode, kp/kd, master gravity + friction compensation (gravity comp makes
  the master weightless and backdrivable: MIT with kp=kd=0), follower gravity
  compensation and velocity feedforward. Apply writes parameters to all
  configured controller nodes and **verifies by readback**, with the last
  result pinned under the button. A hint under Follower kp warns when the
  entered kp has no deflection calibration (the controllers reject it).
- **kp Calibration** (`components/calibration.py`, new): runs
  `calibration/calibrate_kp.py` for a side+kp (~10 min) with progress log and
  Abort; requires control mode Stop, no recording, no scripted motion. Low kp
  (< 25) first enables and resets only the selected follower to a
  high-clearance pose.
- **Calibration store viewer** (same file): read-only view of calibrated kps,
  friction, and gravity scales per side.
- **Scripted Motion** (`components/scripted_motion.py`, new): scenario picker
  from the `robot_eval` registry (sine / stress waypoints / circle) with
  per-scenario speed controls and a "Record motion" checkbox. Recorded runs
  use a ready/trigger file handshake so motion starts only after the recorder
  is live, and auto-stop the episode when the trajectory ends.
- **Static-TF directory sync**: the episode panel's TF directory selection is
  pushed through the `/set_static_transforms` service, fingerprint-cached,
  re-pushed if the TF publisher restarts, and locked while recording.
- **Recording hardening** (`components/recording.py`, refactored out of
  main_control): Start retries within the click; stop/finalize catches
  metadata-snapshot exceptions and force-closes the episode instead of
  leaving an orphan that blocks all future recording; follower
  gravity-compensation CSVs are routed into the episode directory.
- **Episode metadata snapshots**: `episode_meta.json` gains `mit_params`
  (recording-start snapshot of all MIT/compensation parameters from every
  controller node, per-node confirmed/error status) and `ui_params` (panel
  state, scripted-motion settings) — enough to reconstruct what configuration
  produced the data.

### Design rules introduced with these features

- **The launch file owns controller defaults; Apply is the only runtime write
  path.** The app never pushes calibration coefficients — controllers load
  the store themselves and reject uncalibrated kps; the app only surfaces
  that contract.
- **Apply reads widget state at click time and verifies by readback** (a
  render-time-kwargs version silently applied stale panel state; a missed
  failure toast once let a run proceed on the previous config).
- **Every path that could fight the operator stops scripted motion first**:
  Stop, Takeover, arm Disable, arm Reset, and Inference Start all kill the
  scripted-motion subprocess before acting; during a calibration only Stop is
  enabled.

### Tests

```bash
cd python/robo_orchard_inference_app && pytest tests/
```

New tests cover launch-config contracts, generated task configs, MIT
apply/reset flows, recording finalize, static-transform sync, and
scripted-motion integration.
