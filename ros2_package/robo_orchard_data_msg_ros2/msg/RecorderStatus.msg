string IDLE = "idle"
string WAITING = "waiting"
string RECORDING = "recording"
string FINALIZING = "finalizing"
string COMPLETED = "completed"
string FAILED = "failed"

# Snapshot publication time. Terminal states persist until the next Start.
std_msgs/Header header
string data
# UUIDv4 for the current/latest session; empty after node startup.
string session_id
string destination
# Topics from which a message is still required before writing can begin.
string[] waiting_topics
# Nonempty for FAILED, including close failures.
string failure_reason
