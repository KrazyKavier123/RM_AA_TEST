# MCU radians, positive pitch points up. Stamp is HOST RECEIVE time, not exposure time.
std_msgs/Header header
float64 yaw
float64 pitch
float64 roll
float64 yaw_velocity
float64 pitch_velocity
float64 roll_velocity
bool velocity_valid
uint8 status
uint8 is_far
float64 prediction_bias_s
