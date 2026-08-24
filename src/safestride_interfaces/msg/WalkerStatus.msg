# High-level controller/link state.
uint8 STATE_DISCONNECTED=0
uint8 STATE_DISARMED=1
uint8 STATE_ARMED=2
uint8 STATE_SAFE_STOP=3
uint8 STATE_ESTOP=4
uint8 STATE_FAULT=5

# Common fault-bit assignments. Firmware may reserve higher bits for
# board-specific faults.
uint16 FAULT_NONE=0
uint16 FAULT_COMMUNICATION=1
uint16 FAULT_MOTOR_DRIVER=2
uint16 FAULT_LEFT_ENCODER=8
uint16 FAULT_RIGHT_ENCODER=16
uint16 FAULT_OVERCURRENT=32
uint16 FAULT_UNDERVOLTAGE=64
uint16 FAULT_OVERTEMPERATURE=128
uint16 FAULT_PROTOCOL=256

std_msgs/Header header

uint8 state
bool link_ok
bool armed
bool estop
bool watchdog_timeout
bool deadman

uint16 fault_bits

# session_id changes when the Raspberry Pi bridge starts a new link session.
# boot_id changes whenever the microcontroller reboots.
uint32 session_id
uint32 boot_id
uint32 crc_error_count
uint32 frame_error_count

uint16 last_applied_command_sequence

# Age of the underlying MCU telemetry when this status was published, seconds.
float32 telemetry_age
