# Atomic full command. Nonzero header stamp is required; frame_id must be gimbal_base.
std_msgs/Header header
uint8 NO_TARGET=0
uint8 STABLE=1
uint8 UNSTABLE=2
uint8 target_id
uint8 tracking_state
float64 target_yaw
float64 target_pitch
float64 yaw_velocity
float64 pitch_velocity
float64 distance_m
float64 center_x_m
float64 center_y_m
float64 processing_latency_ms
bool fire_allowed
