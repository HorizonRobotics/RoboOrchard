# Robot evaluation scenarios

Scripted hardware evaluations use one reviewed scenario registry and one ROS
executor. Scenario definitions own trajectories, motion limits, pass-through
waypoints, UI defaults, and safety provenance. Do not duplicate those values
in app config, shell scripts, or the executor.

## Commands

```bash
robot-eval list
robot-eval show stress
robot-eval run stress --speed-scale 0.3 --laps 1
robot-eval plan circle --side left
robot-eval plan circle --side right
robot-eval run circle --speed-scale 1.0
robot-eval analyze circle --side left
```

Every run writes the common, line-buffered CSV schema under
`/data/holobrain/robot_eval` unless `--result-path` overrides it. Rows contain
scenario/mode identity, run and trajectory time, phase, and commanded and
measured joints for both arms. Partial runs remain readable after an abort.

## Ownership

- `scenarios/*.json`: reviewed behavior and safety metadata.
- `schema.py`: validation and ROS parameter normalization.
- `cli.py`: the `list`, `show`, `plan`, `run`, and `analyze` interface.
- `results.py`: common result schema.
- `scripted/joint_master.py`: deterministic ROS execution only.
- `circle.py`: circle planner and analyzer.
- `scripted/recorded_replay_master.py`: separate data-driven replay workflow.

To add a scenario, add and test one JSON definition. The inference app
enumerates the registry automatically. A new trajectory mode requires an
executor implementation, but scenario-specific constants still belong in the
JSON definition.
