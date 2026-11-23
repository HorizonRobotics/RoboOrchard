# RoboOchard Deploy ROS2 Package

## Observation-relative pose action channels

`kind: "pose_command"` selects `PoseCommandChannel`, independently of
`JointCommandChannel`. It reads finite `[T, 7]` model rows ordered xyz in
meters followed by quaternion wxyz, and publishes one
`nav_msgs/msg/Path` containing exactly one `PoseStamped` per control tick
on the configured `topic`.
`server_output_key` selects the model response field. For example:

```json
{
  "kind": "pose_command",
  "server_output_key": "mobile_traj",
  "topic": "/base_pose_target",
  "frame_id": "base_link"
}
```

Poses map body coordinates into the fixed body frame at the observation
origin, named by the required `frame_id` in both headers. Each Path carries
that original observation reference time in `header.stamp`, identical for
every row of the chunk. The reference is the latest ROS message header stamp in the
approximately synchronized observation snapshot. It is an approximate common
origin, not an exact simultaneous acquisition time. It is never inference
response time or command publication time. References distinguish observation
origins, not unique chunk IDs: snapshots with equal timestamps remain equal.
Quaternion values are normalized and reordered into ROS xyzw fields; zero
quaternions are rejected. Raw response arrays are not modified.

The single element `poses[0].header.stamp` is the original observation time
plus `round((i + 1) * 1e9 / control_frequency)` nanoseconds, where `i` is the
zero-based row index in the original model response. Async skipped rows and
pending handovers retain this index. This inner stamp describes model sample
time, not publication time or a new coordinate reference. A consumer computes
velocity differences using inner stamp differences, including when intermediate
messages were dropped. The outer stamp alone selects the observation frame.
This is the pose-channel contract; arbitrary multi-pose navigation Paths are
not accepted by the HexFellow tracker. Scheduling remains owned by Deploy.

Migration: change explicit pose-channel and Manager `msg_type` values from
`geometry_msgs/msg/PoseStamped` to `nav_msgs/msg/Path` and upgrade the tracker
with Deploy. The channel `kind`, model field names and HTTP payload stay the
same. Default `msg_type` selection uses Path automatically.

The reference timestamp remains local to the exact inference request, including
asynchronous stitching, pending handovers, and stale-response rejection. No
timestamp, header, or metadata is added to the model-service HTTP payload.

`control_config.control_frequency` is the input-sampling and execution
frequency, **25 Hz by default**. Set `200.0` explicitly for models sampled at
200 Hz, as the HoloBrain project generators do. Explicit values are unchanged.
There is no resampling or separate model/action frequency; model samples must match
this rate. Row i represents the future pose at `(i + 1) / control_frequency`
relative to the observation origin. Deployment preserves existing scheduling,
not coordinated wall-clock start times.

All channels must supply the same nonzero horizon. Deploy validates every
pose row before accepting a new chunk. An invalid pose row, including a zero
quaternion, rejects the entire new chunk without interrupting an active one.
Each channel publishes its row at the current step of an accepted chunk.
Joint command step limiting, trajectory stitching, and `server_remaining_key`
RTC tails apply only to joint channels.

Async scheduling retains the existing RTC-based index policy: progress while a
request is in flight skips that many samples when a remaining-action field was
sent. Without remaining actions, a new chunk starts at row zero. There is no
additional wall-clock/inference-latency correction for pose chunks.
Numerically identical responses are still fresh chunks.

Deploy does not convert poses to velocities, track odometry, project planar
axes, or implement robot control. The previous `TwistCommandChannel` and
`pose_to_twist` helper are removed; direct velocity model outputs are no longer
a Deploy channel. Control Manager's Twist routing and teleoperation are
unaffected. Use a downstream pose-tracking controller for velocity-driven
hardware. The optional [HexFellow adapter](../robo_orchard_hexfellow_ros2/)
provides this controller and launches the official chassis/lift drivers:
`mobile_traj` → Deploy single-pose `Path` → Control Manager → tracker → official
driver `Twist`. It does not change model HTTP or add a shared start-time
protocol; the tracker uses odometry at the original observation stamp.

No zero/identity pose is emitted on exhaustion, disable, invalidation,
publication failure, or shutdown: that would be a motion target, not a stop.
Disable clears buffered actions and invalidates outstanding requests under the
publication lock; resume cannot replay them. Controllers must stop based on
fresh Deploy enabled status, Control Manager routing state, and target freshness;
chunk exhaustion requires a command timeout. Configure hardware safety
independently.

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
