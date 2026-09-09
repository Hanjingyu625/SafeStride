# Atomic supervised command. Only the supervisor publishes this topic.
uint8 DRIVE=0
uint8 BRAKE=1
std_msgs/Header header
float32 target_linear_m_s
int16 slope_ff_pwm
uint8 drive_pwm_cap
uint8 mode
