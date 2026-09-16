# RoboOrchard Inference App

Including visualization, inference, dagger and data recording.

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
older Deploy without the status topic shows Unknown, not Disabled. Existing
control-mode services, node-presence checks, parallel hardware reset, and
Recorder behavior remain in use; Control Manager is not required.
