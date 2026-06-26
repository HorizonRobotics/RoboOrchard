# Full body tracking data from XRoboToolkit/Pico.
# Joint order is the SDK body tracking order. Each pose row is:
# x, y, z, qx, qy, qz, qw.

int32 POSE_WIDTH=7

bool available
int32 joint_count

# Flattened shape [joint_count, 7].
float32[] poses
