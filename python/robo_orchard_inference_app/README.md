# RoboOrchard Inference App

Including visualization, inference, dagger and data recording.

## Control Manager integration

The control buttons send Trigger requests to one global Control Manager:
`/robot/control/auto`, `/robot/control/takeover`, `/robot/control/stop`, and
`/robot/control/reset`. The App reads `ControlMode` from
`/robot/control/status` to display Auto, TakeOver, Stop, or Resetting.
Missing, disconnected, stale, or unrecognized status is shown as Unknown.
Freshness uses the same local receive-time `status_timeout_s` as inference
and Recorder, with independent caches. Service replies acknowledge requests;
they never replace reported control state.

Manager owns command routing and reset sequencing: stop forwarding commands,
disable configured inference services, reset configured hardware, then remain
in STOP. The App does not call driver enable/reset services or disable
inference as part of Reset. Reset is available in AUTO, TAKEOVER and STOP,
but stays locked during an active or pending recording. STOP gates commands;
it does not disable hardware, and TAKEOVER is not hardware teaching mode.

Regenerate the HoloBrain launch configuration using the existing project
scripts. They now start Manager instead of the muxer and select robot wiring
through `teleop/gen_control_manager_config.py`; the App workflow and buttons
stay the same. For custom App configurations, replace `release_service_name`
with `auto_service_name`, configure `reset_service_name` for Manager, and move
the old `enable_arm_service_name` / `reset_arm_service_name` lists into the
Manager's `enable_services` / `reset_services`. Robot-specific behavior
remains in the drivers. `control_status_topic` is configurable, and
`reset_timeout_s` defaults to 180 seconds for the aggregate reset request.
Build Manager and the updated `robo_orchard_teleop_msg_ros2` interfaces and
make those interfaces available to rosbridge. Old per-arm orchestrators are
not used by this configuration.

## Inference runtime status

The App reads Deploy's `InferenceStatus` topic to display Enabled, Disabled,
or Unknown. Enabled means inference is permitted; it does not guarantee
available observations, successful model responses, or robot motion.

The state panel refreshes every second. `ROSBridgeCfg.inference_status_topic`
defaults to `/robot/inference_service/status`; override it for a different
Deploy namespace. `status_timeout_s` defaults to 5 seconds and must be positive
and finite. Freshness uses the local receive time, so ROS and App clocks need
not be synchronized. Missing, disconnected, expired, or unrecognized status
is shown as Unknown. After reconnecting, the App waits for a new snapshot.

Enable/disable service responses acknowledge the request; they do not update
the displayed inference state. External clients can change that state and the
App reflects the next received snapshot. Instruction setup before enable and
static-transform synchronization remain unchanged.

Build and deploy `robo_orchard_deploy_msg_ros2` with the updated Deploy node,
and make its interfaces available to rosbridge. Connecting this App to an
older Deploy without the status topic shows Unknown, not Disabled. Deploy
still owns inference enable/disable; Manager separately selects whether its
output or human commands may reach the drivers.

## Recorder lifecycle

The App reads `<recorder_name>/status`
(`robo_orchard_data_msg_ros2/msg/RecorderStatus`) to display `idle`, `waiting`,
`recording`, `finalizing`, `completed` and `failed`. Current State has three
columns: Control, Recording and Inference, with an English heading above each
status indicator. Recorder details (destination, missing topics, failure reason
and pending operations) appear below those indicators, not in Data Recorder.
Data Recorder retains only its Start/Stop controls. Both panels refresh every
second.
Missing, disconnected or stale status is shown as unknown; Start/Stop are
disabled until a valid snapshot arrives. `status_timeout_s` defaults to 5 seconds
and uses local monotonic receive time, independently of the ROS timestamp.

Recorder owns waiting for data, writing and closing files. The App sends the
existing Start/Stop service requests; their responses do not replace node status.
Before sending Start, the App locks Reset, TF configuration, and repeated Start
until a matching session status arrives. This local pending guard does not
declare the Recorder to be recording; old or unrelated snapshots cannot release
it. Reset checks the guard again in its callback, not only when drawing buttons.
Episode data and log directories retain the `episode_YYYY_MM_DD-HH_MM_SS`
format. Start requests that would reuse a timestamp or an existing path are
rejected locally; retry in the next second. Rejected service requests do not
release the allocated timestamp, so delayed status cannot match a later Start.
It no longer polls `__RECORDING__` or infers completion from local files.
An explicit Stop only finalizes an episode after a matching
`session_id` and absolute destination report `completed`. Repeated heartbeats
cannot repeat counters or metadata writes. Empty cancellations, failed sessions,
and completions belonging to other clients do not count as successful episodes.
Any pending completion is settled before preparing the next recording path.
Metadata is written atomically before incrementing the episode counter. Missing
directories or write failures keep settlement pending and block the next Start;
panel refreshes retry the write without sending another Stop. Once completion
is confirmed, metadata retries do not depend on later Recorder heartbeats and
do not imply that the node is still recording.
New metadata files use normal creation permissions subject to the process
umask; replacing existing metadata preserves its permission bits.
Retries preserve the original metadata and counter despite later UI edits.
Successfully deleting that episode in the App cancels its pending settlement
without counting it or leaving Start blocked.
Rejected or undispatched Start requests restore the previous local paths. A
timeout or other unknown outcome keeps the pending destination and control
guard; matching Recorder status binds its session, even if it arrives before
the RPC returns. No automatic retry is sent. Stop settlement requires both that
session and destination. Other service helpers retain their Boolean results;
`start_recording()` and `stop_recording()` return `None` for an unknown
dispatched result. Stop timeouts retain the pending session; matching terminal
status, not the missing RPC reply, determines settlement. The bridge retains
one requested Stop's terminal result until the App consumes it, so another
client's newer session or a disconnect cannot overwrite received completion
evidence. This result is separate from the live, freshness-checked display.
Deleting a directory decrements a counter only if this App counted that recording;
cancelled, failed or externally completed recordings do not subtract successes.

App cleanup releases subscriptions but does not stop a recording; the Recorder
continues independently. Use Stop explicitly before exiting if desired.
This UI requires the upgraded Recorder and generated messages; older nodes are
shown as unknown rather than using a file-polling fallback. Recorder does not
depend on Manager or Deploy status to manage its lifecycle. Hand-eye and
static-transform behavior is unchanged.
