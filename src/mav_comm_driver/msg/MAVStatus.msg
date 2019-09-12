Header header
uint8 mode_id

# formatted sensor data
sensor_msgs/Imu imu_data
nav_msgs/Odometry odom

# PWM output value
uint8 mid_servo_pwm
uint8 left_servo_pwm
uint8 right_servo_pwm
uint8 throttle_pwm
uint8 climb_pwm

# system status
uint8 sys_status

# origin data
float32 yaw_angle
float32 pitch_angle
float32 roll_angle
float32 yaw_rate
float32 pitch_rate
float32 roll_rate
uint32 board_time # us

# tuning mode value
uint8 pid_id
float32 pid_ext_err # degree * 100
float32 pid_int_err # degree / s * 10
float32 ext_p_output
float32 ext_i_output
float32 ext_d_output
float32 int_p_output
float32 int_i_output
float32 int_d_output


# modes
uint8 FAULT_MODE = 0 # 0x00
uint8 START_MODE = 8 # 0x08
uint8 MANUAL_MODE = 16 # 0x10
uint8 FLIGHT_MODE = 24 # 0x18
uint8 TUNING_MODE = 56 # 0x38