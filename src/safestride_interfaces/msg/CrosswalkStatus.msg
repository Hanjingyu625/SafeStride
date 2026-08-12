std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_APPROACHING=1
uint8 STATE_WAIT_AT_CURB=2
uint8 STATE_ENTRY_ALLOWED=3
uint8 STATE_CROSSING=4
uint8 STATE_CROSSING_URGENT=5
uint8 STATE_EXITING=6

uint8 state
bool gps_valid
bool signal_valid
bool entry_allowed
bool urgent
float32 edge_distance_m
float32 progress_m
float32 remaining_m
float32 signal_remaining_s
float32 required_entry_s
float32 crossing_eta_s
float32 target_speed_mps
string command_mode
string reason
string intersection_id
uint32 crosswalk_index
