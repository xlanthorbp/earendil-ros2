std_msgs/Header header

uint8 FRONT_LEFT=0
uint8 FRONT_RIGHT=1
uint8 REAR_LEFT=2
uint8 REAR_RIGHT=3

int8 REVERSE=-1
int8 NEUTRAL=0
int8 FORWARD=1

uint8 wheel
string wheel_name
float32 rpm
int8 direction
string direction_label
float32 target_rpm
int32 drive_duty
uint8 commutation_phase
bool speed_mode
bool brake_active
uint16 fault_code
uint8 hall_state
int32 target_pwm
int32 applied_pwm
uint32 dropped_commands
uint64 received_bytes
uint32 age_ms
bool fresh
bool valid
