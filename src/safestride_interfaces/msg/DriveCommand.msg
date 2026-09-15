# Atomic supervised command. Only the supervisor publishes this topic.
uint8 DRIVE=0
uint8 BRAKE=1
uint8 TERRAIN_STOP=2
# TERRAIN_STOP fades the MCU's actual PWM over 3 seconds, then holds BRAKE.
std_msgs/Header header
float32 target_linear_m_s
# Display only; the controller consumes target_linear_m_s.
float32 target_speed_kmh
int16 slope_ff_pwm
uint8 drive_pwm_cap
uint8 mode
