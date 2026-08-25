std_msgs/Header header

# Signed cumulative wheel positions. A single-channel Hall sensor cannot
# determine direction; firmware applies the shared driver command direction.
int32 left_pulses
int32 right_pulses

float32 left_velocity_rad_s
float32 right_velocity_rad_s
bool calibrated

# In the single-Hall build, right-side pulse/velocity fields mirror the left
# measurement as a common drivetrain estimate; they are not independent data.
bool left_sensor_present
bool right_sensor_present
