# Event types
string TAKEOVER_TRIGGERED = "takeover_triggered"
string REPLAY_COMMAND_SENT = "replay_command_sent"
string RELEASE_TRIGGERED = "release_triggered"
string STOP_TRIGGERED = "stop_triggered"

# The timestamp of when the event occurred
std_msgs/Header header

# The type of the event, e.g., "takeover_triggered"
string event_type

# Optional details about the event
string details