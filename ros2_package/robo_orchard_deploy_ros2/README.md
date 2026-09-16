# RoboOchard Deploy ROS2 Package

## Published inference state

Both nodes publish `robo_orchard_deploy_msg_ros2/msg/InferenceStatus` on the
relative topic `status`, using queue depth 10. A snapshot is published at
startup, after each actual enable/disable transition, and every second. The
header timestamp is the snapshot publication time, including heartbeats.

- `enabled`: inference and action execution are permitted. This does not
  guarantee observations, a model response, available actions, or robot motion.
- `disabled`: inference and action execution are not permitted. The synchronous
  node starts in INIT and the asynchronous node starts paused; both report
  `disabled`. Synchronous IDLE and EXECUTING both report `enabled`.

`InferenceEvent` messages are published on the relative topic `events`, also
with depth 10. Only actual lifecycle transitions produce `enable_triggered` or
`disable_triggered` events; startup, heartbeats, and redundant calls do not.
`details` is explanatory text, not a machine-readable state contract.

For a node launched in `/robot/inference_service`, the topics are
`/robot/inference_service/status` and `/robot/inference_service/events`.
Clients should derive current state from fresh status snapshots, not service
success callbacks or event history. These topics use volatile durability, so
late subscribers obtain a snapshot on the next heartbeat. Existing `enable`
and `disable` Trigger services remain unchanged. Build and deploy the new
message package alongside the node before subscribing through rosbridge.

## Inference enable/disable boundary

Disabling inference invalidates outstanding model requests and clears buffered
actions, pending handovers, and the limiter/stitcher reference state. Repeated
disable calls also invalidate and clear this state. Re-enabling cannot make a
response from an earlier enabled period valid again, including the first
request before any action chunk has been installed.

The asynchronous node checks validity after model inference and again after
stitching, without holding the control lock during either expensive operation.
A reset invalidates in-flight stitch solves; results rejected by the node are
also discarded if their solve began after that reset. The live trajectory is
not replaced by a rejected solve.

Action publication is serialized with disable. A successful disable response
means no previously selected local action can subsequently begin publication.
It does not cancel remote model computation, retract commands already sent to
ROS, or provide a hardware emergency stop.

## Named joint observations

Each `JointStateChannel` keeps its existing `server_input_key`. Its value is
now a JSON form field containing only the source ROS message's `name` and
`position` lists, rather than a binary NumPy file. For example, a channel
selecting two joints can send:

```json
{"name": ["left_joint1", "left_gripper"], "position": [0.12, 0.03]}
```

Names and positions have the same length and order. With `joint_names=None`,
the codec copies the message's published order. An explicit `joint_names`
selection filters/reorders both lists together without renaming joints.
Empty/duplicate names, mismatched lengths, and non-finite positions reject
the observation frame. No header, velocity, effort, or other ROS metadata is
sent. Units remain those of the source topic.

The driver/project owns robot joint names. Deploy does not infer left/right
from topic strings or add model-specific aliases. Piper drivers expose a
`joint_names` list parameter for this purpose; other drivers' existing ROS
names are used directly. Keep command-channel `joint_names` aligned with
the receiving driver and with the server's existing output-array order.

Images, camera-info arrays, and RTC remaining-action arrays retain their
existing field names and NumPy file encoding. `instruction`, `delay_horizon`,
response action arrays, and inference/stitching behavior are unchanged.
Requests containing only joint observations do not require a binary file.
RTC keys must not overlap observation keys, other RTC keys, or the reserved
`instruction`/`delay_horizon` fields. Invalid bindings fail at configuration
load rather than overwriting an observation during request assembly.

### Model-service compatibility

This changes the joint-observation wire format: the server must parse the
JSON form field under each existing joint-observation key instead of loading
a NumPy file for that key. Reconstruct arrays from `position` and use `name`
for joint identity; observation order does not redefine the configured
control-channel response order. Upgrade the client and server together.

The earlier, unmerged `server_joint_names` configuration and top-level
`joint_names`, `action_joint_names`, and `remaining_action_joint_names` form
fields are removed. Joint channel configurations reject unknown fields
instead of silently accepting the removed aliases. There is no new
`server_config` or generic metadata layer.

## Asynchronous trajectory handover

When trajectory stitching is enabled, the node solves for a future control
step and keeps publishing the old chunk until that step. The lead time is
the previous solve duration plus one control period, limited by the available
actions and `max_delay_horizon`. Inference waits while a solved handover is
pending, so its trajectory cannot be overwritten before the switch.

If the solve misses its planned step, the node retains the current chunk
instead of switching at a point that was not constrained by the solver.
With stitching disabled, responses continue to switch without this wait.

A moving handover must occur before the old chunk runs out. Once it is
exhausted, the next solve starts from its final joint-position commands with
zero velocity and acceleration, rather than extrapolating its old motion.
These are planned command states, not measurements of the robot's physical
position or velocity; downstream command limiting and robot tracking can
make them differ from the actual motion.

Short tails of one to three control steps use the same solver and keep
their requested starting index. The solver has its own integration grid;
there is no four-action-sample requirement and no need to move the start
backwards or skip stitching solely because the tail is short.
