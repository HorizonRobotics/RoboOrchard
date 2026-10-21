# Source-owned readiness and initial-engagement matching heartbeat.
# See README.md for freshness, matching, and rearm semantics.
std_msgs/Header header

# Fully resolved JointState candidate topic. When ready, header.stamp must
# equal the stamp of the candidate evaluated by this matching result.
string candidate_topic

# Increment on each observed ready-to-unavailable transition, including losses
# recovered between publications. Retain the value in every later heartbeat.
uint64 loss_id

# True only when the source input, robot feedback, and driver are usable.
bool ready

# True only when ready and the source's initial match condition is satisfied.
# A mismatch after engagement does not by itself disengage teleoperation.
bool matched

# Human-readable readiness/matching explanation; not a machine state code.
string reason
