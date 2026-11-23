# Runtime and Inference App Boundary

ROS nodes own runtime behavior and expose it through ROS services and status
topics, independently of whether an Inference App or another client is running.

- Control Manager owns command routing, control modes, and reset sequencing.
  Declared Twist channels neutralize on routing transitions and never replay
  historical velocities. Path targets forward unchanged, preserving their
  observation reference header and inner pose sample times. Paths and
  PoseStamped targets are never cached or replayed.
  Optional autonomous/override inputs allow separate typed one-sided routes;
  each route must have at least one source. Takeover replay remains all-or-none
  across eligible channels with an autonomous input, excluding Twist,
  PoseStamped and Path. Override-only routes do not block joint replay.
  Manager never synthesizes a stop pose; downstream controllers cancel tracking using fresh
  control/inference state and target freshness.
- Deploy owns observation processing, model requests, action publication, and
  inference enable/disable behavior. Observation-relative Path targets
  retain the exact request's local observation reference timestamp in the
  outer header; each Path contains one pose stamped with its model sample
  time derived from the original row index and configured control frequency. Deploy
  does not implement pose tracking or convert targets to robot velocities.
  Downstream controllers own feedback and stopping based on fresh Deploy
  enabled state, routing state, and target freshness/command timeout; a zero
  pose is not a stop command.
- Recorder owns waiting for data, writing and closing recordings, and reporting
  session status, completion, and failure reasons.

Inference App owns frontend configuration, operation requests, status display,
and episode metadata and counters. It reads node status to display runtime
state, including unknown/offline when status is unavailable. A service response
acknowledges an operation; it does not replace the node's reported state.
Do not move runtime sequencing, recording readiness, or file-based completion
detection into the App. Repeated status messages must not repeat episode
bookkeeping.

Projects own launch wiring and robot-specific Control Manager configuration.
Deploy channel configuration separately defines observation/action mappings
and the model-service contract. Keep shared nodes generic and use existing
project configuration generators where available.
