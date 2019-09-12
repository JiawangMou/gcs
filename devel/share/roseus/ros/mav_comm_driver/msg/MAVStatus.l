;; Auto-generated. Do not edit!


(when (boundp 'mav_comm_driver::MAVStatus)
  (if (not (find-package "MAV_COMM_DRIVER"))
    (make-package "MAV_COMM_DRIVER"))
  (shadow 'MAVStatus (find-package "MAV_COMM_DRIVER")))
(unless (find-package "MAV_COMM_DRIVER::MAVSTATUS")
  (make-package "MAV_COMM_DRIVER::MAVSTATUS"))

(in-package "ROS")
;;//! \htmlinclude MAVStatus.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*FAULT_MODE*" (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(shadow '*FAULT_MODE* (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(defconstant mav_comm_driver::MAVStatus::*FAULT_MODE* 0)
(intern "*START_MODE*" (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(shadow '*START_MODE* (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(defconstant mav_comm_driver::MAVStatus::*START_MODE* 8)
(intern "*MANUAL_MODE*" (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(shadow '*MANUAL_MODE* (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(defconstant mav_comm_driver::MAVStatus::*MANUAL_MODE* 16)
(intern "*FLIGHT_MODE*" (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(shadow '*FLIGHT_MODE* (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(defconstant mav_comm_driver::MAVStatus::*FLIGHT_MODE* 24)
(intern "*TUNING_MODE*" (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(shadow '*TUNING_MODE* (find-package "MAV_COMM_DRIVER::MAVSTATUS"))
(defconstant mav_comm_driver::MAVStatus::*TUNING_MODE* 56)
(defclass mav_comm_driver::MAVStatus
  :super ros::object
  :slots (_header _mode_id _imu_data _odom _mid_servo_pwm _left_servo_pwm _right_servo_pwm _throttle_pwm _climb_pwm _sys_status _yaw_angle _pitch_angle _roll_angle _yaw_rate _pitch_rate _roll_rate _board_time _pid_id _pid_ext_err _pid_int_err _ext_p_output _ext_i_output _ext_d_output _int_p_output _int_i_output _int_d_output ))

(defmethod mav_comm_driver::MAVStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode_id __mode_id) 0)
    ((:imu_data __imu_data) (instance sensor_msgs::Imu :init))
    ((:odom __odom) (instance nav_msgs::Odometry :init))
    ((:mid_servo_pwm __mid_servo_pwm) 0)
    ((:left_servo_pwm __left_servo_pwm) 0)
    ((:right_servo_pwm __right_servo_pwm) 0)
    ((:throttle_pwm __throttle_pwm) 0)
    ((:climb_pwm __climb_pwm) 0)
    ((:sys_status __sys_status) 0)
    ((:yaw_angle __yaw_angle) 0.0)
    ((:pitch_angle __pitch_angle) 0.0)
    ((:roll_angle __roll_angle) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:pitch_rate __pitch_rate) 0.0)
    ((:roll_rate __roll_rate) 0.0)
    ((:board_time __board_time) 0)
    ((:pid_id __pid_id) 0)
    ((:pid_ext_err __pid_ext_err) 0.0)
    ((:pid_int_err __pid_int_err) 0.0)
    ((:ext_p_output __ext_p_output) 0.0)
    ((:ext_i_output __ext_i_output) 0.0)
    ((:ext_d_output __ext_d_output) 0.0)
    ((:int_p_output __int_p_output) 0.0)
    ((:int_i_output __int_i_output) 0.0)
    ((:int_d_output __int_d_output) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode_id (round __mode_id))
   (setq _imu_data __imu_data)
   (setq _odom __odom)
   (setq _mid_servo_pwm (round __mid_servo_pwm))
   (setq _left_servo_pwm (round __left_servo_pwm))
   (setq _right_servo_pwm (round __right_servo_pwm))
   (setq _throttle_pwm (round __throttle_pwm))
   (setq _climb_pwm (round __climb_pwm))
   (setq _sys_status (round __sys_status))
   (setq _yaw_angle (float __yaw_angle))
   (setq _pitch_angle (float __pitch_angle))
   (setq _roll_angle (float __roll_angle))
   (setq _yaw_rate (float __yaw_rate))
   (setq _pitch_rate (float __pitch_rate))
   (setq _roll_rate (float __roll_rate))
   (setq _board_time (round __board_time))
   (setq _pid_id (round __pid_id))
   (setq _pid_ext_err (float __pid_ext_err))
   (setq _pid_int_err (float __pid_int_err))
   (setq _ext_p_output (float __ext_p_output))
   (setq _ext_i_output (float __ext_i_output))
   (setq _ext_d_output (float __ext_d_output))
   (setq _int_p_output (float __int_p_output))
   (setq _int_i_output (float __int_i_output))
   (setq _int_d_output (float __int_d_output))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode_id
   (&optional __mode_id)
   (if __mode_id (setq _mode_id __mode_id)) _mode_id)
  (:imu_data
   (&rest __imu_data)
   (if (keywordp (car __imu_data))
       (send* _imu_data __imu_data)
     (progn
       (if __imu_data (setq _imu_data (car __imu_data)))
       _imu_data)))
  (:odom
   (&rest __odom)
   (if (keywordp (car __odom))
       (send* _odom __odom)
     (progn
       (if __odom (setq _odom (car __odom)))
       _odom)))
  (:mid_servo_pwm
   (&optional __mid_servo_pwm)
   (if __mid_servo_pwm (setq _mid_servo_pwm __mid_servo_pwm)) _mid_servo_pwm)
  (:left_servo_pwm
   (&optional __left_servo_pwm)
   (if __left_servo_pwm (setq _left_servo_pwm __left_servo_pwm)) _left_servo_pwm)
  (:right_servo_pwm
   (&optional __right_servo_pwm)
   (if __right_servo_pwm (setq _right_servo_pwm __right_servo_pwm)) _right_servo_pwm)
  (:throttle_pwm
   (&optional __throttle_pwm)
   (if __throttle_pwm (setq _throttle_pwm __throttle_pwm)) _throttle_pwm)
  (:climb_pwm
   (&optional __climb_pwm)
   (if __climb_pwm (setq _climb_pwm __climb_pwm)) _climb_pwm)
  (:sys_status
   (&optional __sys_status)
   (if __sys_status (setq _sys_status __sys_status)) _sys_status)
  (:yaw_angle
   (&optional __yaw_angle)
   (if __yaw_angle (setq _yaw_angle __yaw_angle)) _yaw_angle)
  (:pitch_angle
   (&optional __pitch_angle)
   (if __pitch_angle (setq _pitch_angle __pitch_angle)) _pitch_angle)
  (:roll_angle
   (&optional __roll_angle)
   (if __roll_angle (setq _roll_angle __roll_angle)) _roll_angle)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:pitch_rate
   (&optional __pitch_rate)
   (if __pitch_rate (setq _pitch_rate __pitch_rate)) _pitch_rate)
  (:roll_rate
   (&optional __roll_rate)
   (if __roll_rate (setq _roll_rate __roll_rate)) _roll_rate)
  (:board_time
   (&optional __board_time)
   (if __board_time (setq _board_time __board_time)) _board_time)
  (:pid_id
   (&optional __pid_id)
   (if __pid_id (setq _pid_id __pid_id)) _pid_id)
  (:pid_ext_err
   (&optional __pid_ext_err)
   (if __pid_ext_err (setq _pid_ext_err __pid_ext_err)) _pid_ext_err)
  (:pid_int_err
   (&optional __pid_int_err)
   (if __pid_int_err (setq _pid_int_err __pid_int_err)) _pid_int_err)
  (:ext_p_output
   (&optional __ext_p_output)
   (if __ext_p_output (setq _ext_p_output __ext_p_output)) _ext_p_output)
  (:ext_i_output
   (&optional __ext_i_output)
   (if __ext_i_output (setq _ext_i_output __ext_i_output)) _ext_i_output)
  (:ext_d_output
   (&optional __ext_d_output)
   (if __ext_d_output (setq _ext_d_output __ext_d_output)) _ext_d_output)
  (:int_p_output
   (&optional __int_p_output)
   (if __int_p_output (setq _int_p_output __int_p_output)) _int_p_output)
  (:int_i_output
   (&optional __int_i_output)
   (if __int_i_output (setq _int_i_output __int_i_output)) _int_i_output)
  (:int_d_output
   (&optional __int_d_output)
   (if __int_d_output (setq _int_d_output __int_d_output)) _int_d_output)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _mode_id
    1
    ;; sensor_msgs/Imu _imu_data
    (send _imu_data :serialization-length)
    ;; nav_msgs/Odometry _odom
    (send _odom :serialization-length)
    ;; uint8 _mid_servo_pwm
    1
    ;; uint8 _left_servo_pwm
    1
    ;; uint8 _right_servo_pwm
    1
    ;; uint8 _throttle_pwm
    1
    ;; uint8 _climb_pwm
    1
    ;; uint8 _sys_status
    1
    ;; float32 _yaw_angle
    4
    ;; float32 _pitch_angle
    4
    ;; float32 _roll_angle
    4
    ;; float32 _yaw_rate
    4
    ;; float32 _pitch_rate
    4
    ;; float32 _roll_rate
    4
    ;; uint32 _board_time
    4
    ;; uint8 _pid_id
    1
    ;; float32 _pid_ext_err
    4
    ;; float32 _pid_int_err
    4
    ;; float32 _ext_p_output
    4
    ;; float32 _ext_i_output
    4
    ;; float32 _ext_d_output
    4
    ;; float32 _int_p_output
    4
    ;; float32 _int_i_output
    4
    ;; float32 _int_d_output
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _mode_id
       (write-byte _mode_id s)
     ;; sensor_msgs/Imu _imu_data
       (send _imu_data :serialize s)
     ;; nav_msgs/Odometry _odom
       (send _odom :serialize s)
     ;; uint8 _mid_servo_pwm
       (write-byte _mid_servo_pwm s)
     ;; uint8 _left_servo_pwm
       (write-byte _left_servo_pwm s)
     ;; uint8 _right_servo_pwm
       (write-byte _right_servo_pwm s)
     ;; uint8 _throttle_pwm
       (write-byte _throttle_pwm s)
     ;; uint8 _climb_pwm
       (write-byte _climb_pwm s)
     ;; uint8 _sys_status
       (write-byte _sys_status s)
     ;; float32 _yaw_angle
       (sys::poke _yaw_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_angle
       (sys::poke _pitch_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_angle
       (sys::poke _roll_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_rate
       (sys::poke _pitch_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_rate
       (sys::poke _roll_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _board_time
       (write-long _board_time s)
     ;; uint8 _pid_id
       (write-byte _pid_id s)
     ;; float32 _pid_ext_err
       (sys::poke _pid_ext_err (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pid_int_err
       (sys::poke _pid_int_err (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ext_p_output
       (sys::poke _ext_p_output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ext_i_output
       (sys::poke _ext_i_output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ext_d_output
       (sys::poke _ext_d_output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _int_p_output
       (sys::poke _int_p_output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _int_i_output
       (sys::poke _int_i_output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _int_d_output
       (sys::poke _int_d_output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _mode_id
     (setq _mode_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; sensor_msgs/Imu _imu_data
     (send _imu_data :deserialize buf ptr-) (incf ptr- (send _imu_data :serialization-length))
   ;; nav_msgs/Odometry _odom
     (send _odom :deserialize buf ptr-) (incf ptr- (send _odom :serialization-length))
   ;; uint8 _mid_servo_pwm
     (setq _mid_servo_pwm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _left_servo_pwm
     (setq _left_servo_pwm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _right_servo_pwm
     (setq _right_servo_pwm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _throttle_pwm
     (setq _throttle_pwm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _climb_pwm
     (setq _climb_pwm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sys_status
     (setq _sys_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _yaw_angle
     (setq _yaw_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_angle
     (setq _pitch_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_angle
     (setq _roll_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_rate
     (setq _pitch_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_rate
     (setq _roll_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _board_time
     (setq _board_time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _pid_id
     (setq _pid_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _pid_ext_err
     (setq _pid_ext_err (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pid_int_err
     (setq _pid_int_err (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ext_p_output
     (setq _ext_p_output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ext_i_output
     (setq _ext_i_output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ext_d_output
     (setq _ext_d_output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _int_p_output
     (setq _int_p_output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _int_i_output
     (setq _int_i_output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _int_d_output
     (setq _int_d_output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mav_comm_driver::MAVStatus :md5sum-) "b02f3350f8be0b668bda0c83dc2bc899")
(setf (get mav_comm_driver::MAVStatus :datatype-) "mav_comm_driver/MAVStatus")
(setf (get mav_comm_driver::MAVStatus :definition-)
      "Header header
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
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :mav_comm_driver/MAVStatus "b02f3350f8be0b668bda0c83dc2bc899")


