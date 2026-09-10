std_msgs/Header header

uint8 ALERT_NORMAL=0
uint8 ALERT_WARNING=1
uint8 ALERT_HANDS_OFF=2

# WARNING includes one-hand operation; HANDS_OFF means both channels released.

float32 left_raw
float32 right_raw
float32 left_filtered
float32 right_filtered
bool left_present
bool right_present
bool calibrated
uint8 alert
