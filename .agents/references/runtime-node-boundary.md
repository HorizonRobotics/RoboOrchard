# Runtime and Inference App Boundary

ROS nodes own runtime behavior and expose it through ROS services and status
topics, independently of whether an Inference App or another client is running.

- Control Manager owns command routing, control modes, and reset sequencing.
- Deploy owns observation processing, model requests, action publication, and
  inference enable/disable behavior.
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
