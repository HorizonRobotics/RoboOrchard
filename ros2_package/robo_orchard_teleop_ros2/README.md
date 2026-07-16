# robo_orchard_teleop_ros2

Teleoperation package for the dual-arm Piper (ALOHA-style) rig. Upstream
provides the Pico VR bridge (`pico_bridge`, `piper_pico_vr_teleop`), the
takeover muxer (`take_over`), its triggers, and the compat launch files. This
README covers what this branch adds on top.

## Added components

| Component | Entry point | Purpose |
|---|---|---|
| ALOHA orchestrator | `aloha_orchestrator` | Per-side `auto`/`takeover` Trigger services that sequence arm `enable_ctrl` calls with the muxer mode switch, closing the race where the muxer forwarded commands to arms still in teach mode. Stop stays a direct muxer call |
| Scripted joint master | `scripted_joint_master` | Deterministic trajectory publisher (sine / waypoints / circle) for hardware evaluation — see `robo_orchard_teleop_ros2/scripted/README.md` |
| Recorded replay master | `recorded_replay_master` | Path-parameterized replay of a recorded safe trajectory for gravity/system-ID data collection — same README |
| Evaluation registry + CLI | `robot-eval` | Reviewed scenario definitions with `list/show/plan/run/analyze` — see `robo_orchard_teleop_ros2/robot_eval/README.md` |

## Launch changes

- **`piper_dagger_compat.launch.py`** (reworked): upstream started the two
  muxers and four `single_ctrl` nodes with bare MIT toggles. This branch adds
  the per-side ALOHA orchestrators and makes the launch the owner of the
  production MIT configuration: follower kp 25 / kd 0.8, gravity compensation
  through the kp position offset, velocity feedforward with phase lead, and
  the calibration-store path handed to the followers — with **no calibrated
  numbers in the launch file itself** (the store is the single source; the
  app's Apply button is the only other write path). Start via
  `projects/HoloBrain/teleop/dagger.sh`, which also verifies the deflection
  calibration loaded.
- **`scripted_joint_master.launch.py`**, **`recorded_replay_master.launch.py`**
  (new): single-node wrappers for the scripted masters; scenario-driven runs
  normally go through `robot-eval run` or the app.

## Typical operation

```bash
# in the holobrain container
bash /opt/roboorchard/projects/HoloBrain/teleop/dagger.sh   # bring up the stack

# hand control to the human (per side) / back to the algorithm
ros2 service call /robot/left/takeover_muxer/trigger_takeover std_srvs/srv/Trigger
ros2 service call /robot/left/aloha_orchestrator/auto std_srvs/srv/Trigger

# scripted hardware evaluation
robot-eval list
robot-eval run stress --speed-scale 0.3
```

The inference app (`python/robo_orchard_inference_app`) wraps these in a UI:
control-mode buttons, MIT parameter apply, scripted motion with recording,
and kp calibration.
