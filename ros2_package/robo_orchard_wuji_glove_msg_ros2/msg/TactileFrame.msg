std_msgs/Header header
uint64 sequence
uint64 device_timestamp_us
uint16 rows
uint16 columns
# Row-major calibrated pressure/contact values. Invalid taxels are -1.0.
float32[] data
