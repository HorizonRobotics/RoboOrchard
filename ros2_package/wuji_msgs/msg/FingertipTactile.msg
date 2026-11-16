# SDK timestamp and frame ID converted to a ROS header.
std_msgs/Header header
# Identifies the SDK point-force layout.
uint32 info_digest
# Force vector at each SDK-defined sensing point, in N.
geometry_msgs/Vector3[] point_forces
# Sum of all point forces, in N.
geometry_msgs/Vector3 total_force
# Sensor temperature, in degrees Celsius.
float32 temperature
