string ENABLE_TRIGGERED = "enable_triggered"
string DISABLE_TRIGGERED = "disable_triggered"

# Emitted only after an actual lifecycle change completes.
std_msgs/Header header
string event_type
string details
