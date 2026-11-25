# ROS source clock stamp. A zero stamp or nonpositive lifetime is invalid.
std_msgs/Header header
builtin_interfaces/Duration valid_for
uint16 motor_id
uint64 owner
uint8 MIXED=0
uint8 POSITION=1
uint8 VELOCITY=2
uint8 CURRENT=3
uint8 TORQUE=4
uint8 DAMPING=5
uint8 DISSIPATIVE=6
uint8 REGENERATIVE=7
uint8 ELECTROMAGNETIC_BRAKE=8
uint8 mode
# Position mode uses speed_limit_rad_s as an unsigned threshold.
float64 position_rad
float64 velocity_rad_s
float64 speed_limit_rad_s
float64 current_a
float64 torque_nm
float64 feedforward_nm
float64 kp
float64 kd
# Position, velocity or braking current threshold.
float64 current_limit_a
bool release_brake
uint8 ACK_NONE=0
uint8 ACK_TYPE1=1
uint8 ACK_TYPE2=2
uint8 ACK_TYPE3=3
uint8 feedback
