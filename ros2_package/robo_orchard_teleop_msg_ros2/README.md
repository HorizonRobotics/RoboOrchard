# Teleoperation interfaces

This package defines shared ROS 2 messages and services. Existing
`ControlMode` and `TakeOverEvent` interfaces are unchanged.
`TeleopActivationState` adds `PENDING=3` for valid held input awaiting its
activation delay. The engagement contracts below allow independent sources to
produce ordinary `sensor_msgs/msg/JointState` candidates while a separate
coordinator gates their routes to Control Manager overrides. Control Manager
continues to own mode selection and hardware command routing.

## Match heartbeat

`TeleopMatchState` contains a producer-stamped `Header`, `candidate_topic`,
`loss_id`, `ready`, `matched`, and a human-readable `reason`. The source owns
its input mapping, robot feedback validation, driver readiness, matching
tolerance, and stable-match dwell. The coordinator does not interpret robot
joints or tolerances.

- `ready=false` means the source cannot safely supply candidates. It must
  also set `matched=false` and explain the cause in `reason`.
- `ready=true, matched=false` means the source is usable but the operator
  input does not yet match the measured robot state for initial engagement.
- `ready=true, matched=true` permits initial engagement, subject to fresh
  activation, Manager TAKEOVER, and a verified baseline recapture.

Publish periodically, including while unavailable or waiting to match.
Matching gates initial engagement only: normal commanded motion can make
input and feedback differ without disengaging. Loss of readiness or a stale
heartbeat suppresses **all** coordinated routes and invalidates engagement.
`reason` is diagnostic text, not a string to parse for control decisions.

When ready, publish the candidate and its match result with the **same**
`header.stamp`, identifying that publication pair. `candidate_topic` is the
fully resolved topic of that candidate, including namespace and remapping.
It must identify a configured input route. Before first forwarding, the
coordinator requires the cached candidate on that route to have exactly the
positive match heartbeat's stamp. A newer candidate cannot be authorized by
an older match, nor can a newer match authorize an older candidate. Delivery
order across the two topics is not assumed; an incomplete pair keeps every
route closed and remains subject to freshness and cutover timeouts. After
engagement, matching and pair equality do not gate normal commanded motion.

The producer increments `loss_id` on each observed ready-to-unavailable
transition, including invalid input or expiry detected before replacing an
input sample. It retains this counter in every subsequent heartbeat, even if
inputs recover before publication. The coordinator invalidates the session
and requires release/rearm whenever the counter changes. Thus recovery cannot
hide a fault when intermediate heartbeats are skipped by DDS. The counter is
process-local, not a persistent identity for detecting every producer restart.
This message layout change requires rebuilding the interface package and all
producers/consumers together.

## Baseline recapture and candidate cutover

`RecaptureBaseline` has an empty request. It asks the producer to capture its
relative-motion baseline from current source input and measured robot state,
clear smoothing/history, and reseed command generation. It does not move the
robot, activate input, select Manager mode, or start a hardware reset. Missing,
invalid, or stale input/feedback and an in-progress reset must fail the request.
On failure return `success=false`, an explanatory `reason`, an empty
`candidate_topic`, and a zero `stamp_barrier`.

On success return the fully resolved candidate topic (including namespace and
remapping) and an exclusive producer timestamp barrier. The producer must
serialize recapture with candidate generation/publication:

1. Stop old-baseline generation and discard pending old-baseline results.
2. Capture the new baseline, clear filters, and update the measured-state seed.
3. Choose a nonzero barrier at least as large as every stamp that could have
   been published from the old baseline, and return it with `success=true`.
4. Stamp every subsequent new-baseline candidate strictly above that barrier.
   Never publish an old-baseline result above the barrier, even if computation
   began before recapture and completed later.

The coordinator clears cached candidates when requesting recapture. It must
accept a response only for its still-current request and engagement attempt,
within its service timeout, and while all engagement prerequisites remain
valid. The response topic must identify exactly one configured input route;
failure, an unknown topic, or an invalid barrier suppresses all output. A
successful response alone is not permission to forward: wait for a fresh
candidate on that route with `header.stamp > stamp_barrier`, together with
fresh candidates on every other route. Candidates at or below the barrier,
including queued DDS deliveries, are discarded. A post-barrier candidate
received before the response may be retained for verification, but never
forwarded before verification completes.

## Freshness, clocks, and rearming

Candidate and heartbeat headers must be stamped at generation/publication
with a nonzero ROS time, not copied from an old input message. Candidate
stamps must strictly increase per producer. The producer and coordinator must
use the same non-decreasing ROS time domain (synchronized system clocks or
the same simulation clock). Stamp barriers are producer-local ordering
boundaries, not a comparison between different producers' sequence numbers.

The coordinator checks both local monotonic receipt age and ROS header age
against its configured freshness timeout. Reject zero, expired, regressing,
or future stamps beyond the allowed clock skew. A just-delivered old queued
message must not renew freshness. Sources likewise use local monotonic receipt
times to detect missing input/feedback; stale data must not become fresh merely
by republishing a candidate. Observable readiness/stream loss or timestamp
regression/discontinuity invalidates the session and requires fresh inputs and
another recapture. The timestamp protocol does not support retaining an
engagement across a clock reset. It cannot detect a fast producer restart that
preserves readiness, stream freshness, and increasing timestamps; no
transparent restart detection is guaranteed. Deployments must keep clock skew
within the freshness budget.

At startup and after faults (unavailable/stale input, failed/timed-out
recapture, lost candidate stream, or leaving Manager TAKEOVER), discard the
attempt and suppress every route. Require a fresh `TeleopActivationState`
`INACTIVE` heartbeat followed by a fresh `ACTIVE` transition before another
attempt. A held activation on startup or recovery must never auto-engage.
`UNAVAILABLE` is not a release, and `transition_id` alone cannot identify a
publisher across restarts. Normal operator release also suppresses output;
the next activation must pass matching and recapture again. Matching or
readiness recovery does not activate input or request Manager TAKEOVER.

`PENDING` is held input, not `INACTIVE` or `UNAVAILABLE`: suppress output,
preserve a real release already observed, and wait for `ACTIVE`. It never
arms a consumer at startup or after a fault. This allows an ordinary
release, press, hold-delay, activation sequence without a false release
while Grip is held. Existing consumers that do not understand `PENDING`
must treat it as unavailable, never active.

These interfaces define the contract; they do not themselves provide a
producer or coordinator implementation.
