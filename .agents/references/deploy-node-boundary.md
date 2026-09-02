# Deploy Node Boundary

## Scope

This reference applies to `ros2_package/robo_orchard_deploy_ros2/`.

The deploy node is the runtime bridge between a robot's ROS 2 interfaces and
a model inference service. It owns the flow from observations to inference to
control publication, while remaining independent of any specific robot,
controller implementation, or model architecture.

## Data Flow

1. Subscribe to the observation topics declared by the user.
2. Synchronize and decode the received ROS 2 messages.
3. Map each observation channel to its configured model-server input key.
4. Request inference from the configured model service.
5. Validate and sequence the model's action response.
6. Encode and publish each action channel to its configured ROS 2 command
   topic.

The synchronous and asynchronous nodes may schedule this flow differently,
but they must share the same observation, inference, and action contracts.

## Configuration Boundary

- Robot-facing topic names, ROS 2 message types, joint names, and QoS choices
  belong in user configuration, not in deploy-node orchestration code.
- Observation channels expose depth, reliability, durability, and history as
  explicit QoS fields. Their defaults must remain equivalent to the previous
  normal subscription behavior: keep-last depth 10, reliable, and volatile.
- Model-facing request and response field names also belong in channel
  configuration through `server_input_key` and `server_output_key`.
- A channel is the adapter between one model field and one ROS 2 topic. The
  deploy node coordinates channels without assuming a particular embodiment,
  number of arms, joint layout, or controller topic namespace.
- The model service contract and the robot topic contract are independent.
  Changing one side should require an adapter or configuration change, not
  robot-specific branches in the deploy nodes.

## Current Channel Support

Observation channels derive from `ObsChannelBase`. The current implementation
supports:

- `ImageChannel` for image observations.
- `CameraInfoChannel` for camera projection data.
- `JointStateChannel` for measured joint positions.

Action channels derive from `ActionChannelBase`. The current implementation
supports only `JointCommandChannel`, which publishes joint-position commands
using `sensor_msgs/msg/JointState`.

The optional `velocity` and `effort` fields populated by
`JointCommandChannel` are auxiliary fields of that joint-position message;
they do not make the channel a velocity-control or force-control interface.

## Extension Boundary

- Add a new observation representation by deriving a channel from
  `ObsChannelBase`, registering it in the observation discriminated union, and
  adding the corresponding message decoder.
- Add a new control mode by deriving a channel from `ActionChannelBase`,
  registering it in the action discriminated union, and adding the
  corresponding action encoder and publisher behavior.
- Velocity control, effort/force control, Cartesian pose control, trajectory
  control, or other higher-level modes should use distinct action-channel
  kinds with explicit message semantics. Do not overload
  `JointCommandChannel` or infer control mode from a topic name.
- Each new channel must define its units, ordering, shape, timing semantics,
  message type, validation rules, and mapping to the model-server field.
- Keep robot SDK calls and controller-specific behavior outside the generic
  deploy node. Adapt them behind ROS 2 topics and message types.

## Ownership by Module

- `config.py` defines channel schemas and validates configuration contracts.
- `topic_manager.py` resolves ROS 2 message classes and creates topic I/O.
- `codec.py` converts observation messages into model input arrays.
- `obs_manager.py` subscribes, synchronizes, and snapshots observations.
- `model_request.py` owns model-service request and response transport.
- `action_exec.py` validates action sequences and publishes control messages.
- `node/sync_node.py` and `node/async_node.py` orchestrate scheduling and node
  lifecycle without adding embodiment-specific conversions.

## Invariants

- Never hardcode a robot's topic names, message types, or joint names in the
  generic node path.
- Reject missing, malformed, or dimensionally inconsistent model outputs
  before publishing them.
- Do not partially publish a multi-channel action step when the response is
  inconsistent across channels.
- Keep observation decoding and action encoding explicit and testable at the
  channel boundary.
- Preserve the separation between model transport, ROS 2 transport, and robot
  control semantics.
