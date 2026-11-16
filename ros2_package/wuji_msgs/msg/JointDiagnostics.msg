# SDK bus node identifier.
uint8 nid
# Formal URDF joint name mapped from nid and the selected hand side.
string name
# Raw SDK joint status word. The wrapper does not decode it.
uint32 status_word
# SDK phase current, in A.
float32 current
# SDK bus voltage, in V.
float32 bus_voltage
# SDK joint-controller MCU temperature, in degrees Celsius.
float32 mcu_temperature
# Raw SDK uint16 error value. The wrapper does not decode it.
uint16 error_code
# SDK RS485 response rate over the most recent second, from 0 through 100 percent.
uint8 comm_response_rate_pct
# Cumulative SDK RS485 communication timeout count.
uint32 comm_timeout_total
