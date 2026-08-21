# Current availability and activation state of the teleoperation clutch.
std_msgs/Header header

uint8 UNAVAILABLE=0
uint8 INACTIVE=1
uint8 ACTIVE=2

uint8 state

# Increments on every state transition. It restarts when the publisher restarts.
uint64 transition_id
