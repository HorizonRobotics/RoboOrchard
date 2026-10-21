# Current availability and activation state of the teleoperation clutch.
std_msgs/Header header

uint8 UNAVAILABLE=0
uint8 INACTIVE=1
uint8 ACTIVE=2

# Valid held input awaiting activation delay; not a release or a fault.
# Preserves a previously observed release, but never arms a consumer itself.
uint8 PENDING=3

uint8 state

# Increments on every state transition. It restarts when the publisher restarts.
uint64 transition_id
