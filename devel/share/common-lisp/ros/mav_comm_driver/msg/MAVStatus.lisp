; Auto-generated. Do not edit!


(cl:in-package mav_comm_driver-msg)


;//! \htmlinclude MAVStatus.msg.html

(cl:defclass <MAVStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode_id
    :reader mode_id
    :initarg :mode_id
    :type cl:fixnum
    :initform 0)
   (imu_data
    :reader imu_data
    :initarg :imu_data
    :type sensor_msgs-msg:Imu
    :initform (cl:make-instance 'sensor_msgs-msg:Imu))
   (odom
    :reader odom
    :initarg :odom
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (mid_servo_pwm
    :reader mid_servo_pwm
    :initarg :mid_servo_pwm
    :type cl:fixnum
    :initform 0)
   (left_servo_pwm
    :reader left_servo_pwm
    :initarg :left_servo_pwm
    :type cl:fixnum
    :initform 0)
   (right_servo_pwm
    :reader right_servo_pwm
    :initarg :right_servo_pwm
    :type cl:fixnum
    :initform 0)
   (throttle_pwm
    :reader throttle_pwm
    :initarg :throttle_pwm
    :type cl:fixnum
    :initform 0)
   (climb_pwm
    :reader climb_pwm
    :initarg :climb_pwm
    :type cl:fixnum
    :initform 0)
   (sys_status
    :reader sys_status
    :initarg :sys_status
    :type cl:fixnum
    :initform 0)
   (yaw_angle
    :reader yaw_angle
    :initarg :yaw_angle
    :type cl:float
    :initform 0.0)
   (pitch_angle
    :reader pitch_angle
    :initarg :pitch_angle
    :type cl:float
    :initform 0.0)
   (roll_angle
    :reader roll_angle
    :initarg :roll_angle
    :type cl:float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0)
   (pitch_rate
    :reader pitch_rate
    :initarg :pitch_rate
    :type cl:float
    :initform 0.0)
   (roll_rate
    :reader roll_rate
    :initarg :roll_rate
    :type cl:float
    :initform 0.0)
   (board_time
    :reader board_time
    :initarg :board_time
    :type cl:integer
    :initform 0)
   (pid_id
    :reader pid_id
    :initarg :pid_id
    :type cl:fixnum
    :initform 0)
   (pid_ext_err
    :reader pid_ext_err
    :initarg :pid_ext_err
    :type cl:float
    :initform 0.0)
   (pid_int_err
    :reader pid_int_err
    :initarg :pid_int_err
    :type cl:float
    :initform 0.0)
   (ext_p_output
    :reader ext_p_output
    :initarg :ext_p_output
    :type cl:float
    :initform 0.0)
   (ext_i_output
    :reader ext_i_output
    :initarg :ext_i_output
    :type cl:float
    :initform 0.0)
   (ext_d_output
    :reader ext_d_output
    :initarg :ext_d_output
    :type cl:float
    :initform 0.0)
   (int_p_output
    :reader int_p_output
    :initarg :int_p_output
    :type cl:float
    :initform 0.0)
   (int_i_output
    :reader int_i_output
    :initarg :int_i_output
    :type cl:float
    :initform 0.0)
   (int_d_output
    :reader int_d_output
    :initarg :int_d_output
    :type cl:float
    :initform 0.0))
)

(cl:defclass MAVStatus (<MAVStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MAVStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MAVStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mav_comm_driver-msg:<MAVStatus> is deprecated: use mav_comm_driver-msg:MAVStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:header-val is deprecated.  Use mav_comm_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode_id-val :lambda-list '(m))
(cl:defmethod mode_id-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:mode_id-val is deprecated.  Use mav_comm_driver-msg:mode_id instead.")
  (mode_id m))

(cl:ensure-generic-function 'imu_data-val :lambda-list '(m))
(cl:defmethod imu_data-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:imu_data-val is deprecated.  Use mav_comm_driver-msg:imu_data instead.")
  (imu_data m))

(cl:ensure-generic-function 'odom-val :lambda-list '(m))
(cl:defmethod odom-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:odom-val is deprecated.  Use mav_comm_driver-msg:odom instead.")
  (odom m))

(cl:ensure-generic-function 'mid_servo_pwm-val :lambda-list '(m))
(cl:defmethod mid_servo_pwm-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:mid_servo_pwm-val is deprecated.  Use mav_comm_driver-msg:mid_servo_pwm instead.")
  (mid_servo_pwm m))

(cl:ensure-generic-function 'left_servo_pwm-val :lambda-list '(m))
(cl:defmethod left_servo_pwm-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:left_servo_pwm-val is deprecated.  Use mav_comm_driver-msg:left_servo_pwm instead.")
  (left_servo_pwm m))

(cl:ensure-generic-function 'right_servo_pwm-val :lambda-list '(m))
(cl:defmethod right_servo_pwm-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:right_servo_pwm-val is deprecated.  Use mav_comm_driver-msg:right_servo_pwm instead.")
  (right_servo_pwm m))

(cl:ensure-generic-function 'throttle_pwm-val :lambda-list '(m))
(cl:defmethod throttle_pwm-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:throttle_pwm-val is deprecated.  Use mav_comm_driver-msg:throttle_pwm instead.")
  (throttle_pwm m))

(cl:ensure-generic-function 'climb_pwm-val :lambda-list '(m))
(cl:defmethod climb_pwm-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:climb_pwm-val is deprecated.  Use mav_comm_driver-msg:climb_pwm instead.")
  (climb_pwm m))

(cl:ensure-generic-function 'sys_status-val :lambda-list '(m))
(cl:defmethod sys_status-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:sys_status-val is deprecated.  Use mav_comm_driver-msg:sys_status instead.")
  (sys_status m))

(cl:ensure-generic-function 'yaw_angle-val :lambda-list '(m))
(cl:defmethod yaw_angle-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:yaw_angle-val is deprecated.  Use mav_comm_driver-msg:yaw_angle instead.")
  (yaw_angle m))

(cl:ensure-generic-function 'pitch_angle-val :lambda-list '(m))
(cl:defmethod pitch_angle-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:pitch_angle-val is deprecated.  Use mav_comm_driver-msg:pitch_angle instead.")
  (pitch_angle m))

(cl:ensure-generic-function 'roll_angle-val :lambda-list '(m))
(cl:defmethod roll_angle-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:roll_angle-val is deprecated.  Use mav_comm_driver-msg:roll_angle instead.")
  (roll_angle m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:yaw_rate-val is deprecated.  Use mav_comm_driver-msg:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'pitch_rate-val :lambda-list '(m))
(cl:defmethod pitch_rate-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:pitch_rate-val is deprecated.  Use mav_comm_driver-msg:pitch_rate instead.")
  (pitch_rate m))

(cl:ensure-generic-function 'roll_rate-val :lambda-list '(m))
(cl:defmethod roll_rate-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:roll_rate-val is deprecated.  Use mav_comm_driver-msg:roll_rate instead.")
  (roll_rate m))

(cl:ensure-generic-function 'board_time-val :lambda-list '(m))
(cl:defmethod board_time-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:board_time-val is deprecated.  Use mav_comm_driver-msg:board_time instead.")
  (board_time m))

(cl:ensure-generic-function 'pid_id-val :lambda-list '(m))
(cl:defmethod pid_id-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:pid_id-val is deprecated.  Use mav_comm_driver-msg:pid_id instead.")
  (pid_id m))

(cl:ensure-generic-function 'pid_ext_err-val :lambda-list '(m))
(cl:defmethod pid_ext_err-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:pid_ext_err-val is deprecated.  Use mav_comm_driver-msg:pid_ext_err instead.")
  (pid_ext_err m))

(cl:ensure-generic-function 'pid_int_err-val :lambda-list '(m))
(cl:defmethod pid_int_err-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:pid_int_err-val is deprecated.  Use mav_comm_driver-msg:pid_int_err instead.")
  (pid_int_err m))

(cl:ensure-generic-function 'ext_p_output-val :lambda-list '(m))
(cl:defmethod ext_p_output-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:ext_p_output-val is deprecated.  Use mav_comm_driver-msg:ext_p_output instead.")
  (ext_p_output m))

(cl:ensure-generic-function 'ext_i_output-val :lambda-list '(m))
(cl:defmethod ext_i_output-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:ext_i_output-val is deprecated.  Use mav_comm_driver-msg:ext_i_output instead.")
  (ext_i_output m))

(cl:ensure-generic-function 'ext_d_output-val :lambda-list '(m))
(cl:defmethod ext_d_output-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:ext_d_output-val is deprecated.  Use mav_comm_driver-msg:ext_d_output instead.")
  (ext_d_output m))

(cl:ensure-generic-function 'int_p_output-val :lambda-list '(m))
(cl:defmethod int_p_output-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:int_p_output-val is deprecated.  Use mav_comm_driver-msg:int_p_output instead.")
  (int_p_output m))

(cl:ensure-generic-function 'int_i_output-val :lambda-list '(m))
(cl:defmethod int_i_output-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:int_i_output-val is deprecated.  Use mav_comm_driver-msg:int_i_output instead.")
  (int_i_output m))

(cl:ensure-generic-function 'int_d_output-val :lambda-list '(m))
(cl:defmethod int_d_output-val ((m <MAVStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:int_d_output-val is deprecated.  Use mav_comm_driver-msg:int_d_output instead.")
  (int_d_output m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MAVStatus>)))
    "Constants for message type '<MAVStatus>"
  '((:FAULT_MODE . 0)
    (:START_MODE . 8)
    (:MANUAL_MODE . 16)
    (:FLIGHT_MODE . 24)
    (:TUNING_MODE . 56))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MAVStatus)))
    "Constants for message type 'MAVStatus"
  '((:FAULT_MODE . 0)
    (:START_MODE . 8)
    (:MANUAL_MODE . 16)
    (:FLIGHT_MODE . 24)
    (:TUNING_MODE . 56))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MAVStatus>) ostream)
  "Serializes a message object of type '<MAVStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu_data) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'odom) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mid_servo_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_servo_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_servo_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'throttle_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'climb_pwm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sys_status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'board_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'board_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'board_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'board_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pid_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_ext_err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_int_err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ext_p_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ext_i_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ext_d_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'int_p_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'int_i_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'int_d_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MAVStatus>) istream)
  "Deserializes a message object of type '<MAVStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu_data) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'odom) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mid_servo_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_servo_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_servo_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'throttle_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'climb_pwm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sys_status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'board_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'board_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'board_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'board_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pid_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_ext_err) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_int_err) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ext_p_output) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ext_i_output) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ext_d_output) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'int_p_output) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'int_i_output) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'int_d_output) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MAVStatus>)))
  "Returns string type for a message object of type '<MAVStatus>"
  "mav_comm_driver/MAVStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MAVStatus)))
  "Returns string type for a message object of type 'MAVStatus"
  "mav_comm_driver/MAVStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MAVStatus>)))
  "Returns md5sum for a message object of type '<MAVStatus>"
  "b02f3350f8be0b668bda0c83dc2bc899")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MAVStatus)))
  "Returns md5sum for a message object of type 'MAVStatus"
  "b02f3350f8be0b668bda0c83dc2bc899")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MAVStatus>)))
  "Returns full string definition for message of type '<MAVStatus>"
  (cl:format cl:nil "Header header~%uint8 mode_id~%~%# formatted sensor data~%sensor_msgs/Imu imu_data~%nav_msgs/Odometry odom~%~%# PWM output value~%uint8 mid_servo_pwm~%uint8 left_servo_pwm~%uint8 right_servo_pwm~%uint8 throttle_pwm~%uint8 climb_pwm~%~%# system status~%uint8 sys_status~%~%# origin data~%float32 yaw_angle~%float32 pitch_angle~%float32 roll_angle~%float32 yaw_rate~%float32 pitch_rate~%float32 roll_rate~%uint32 board_time # us~%~%# tuning mode value~%uint8 pid_id~%float32 pid_ext_err # degree * 100~%float32 pid_int_err # degree / s * 10~%float32 ext_p_output~%float32 ext_i_output~%float32 ext_d_output~%float32 int_p_output~%float32 int_i_output~%float32 int_d_output~%~%~%# modes~%uint8 FAULT_MODE = 0 # 0x00~%uint8 START_MODE = 8 # 0x08~%uint8 MANUAL_MODE = 16 # 0x10~%uint8 FLIGHT_MODE = 24 # 0x18~%uint8 TUNING_MODE = 56 # 0x38~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MAVStatus)))
  "Returns full string definition for message of type 'MAVStatus"
  (cl:format cl:nil "Header header~%uint8 mode_id~%~%# formatted sensor data~%sensor_msgs/Imu imu_data~%nav_msgs/Odometry odom~%~%# PWM output value~%uint8 mid_servo_pwm~%uint8 left_servo_pwm~%uint8 right_servo_pwm~%uint8 throttle_pwm~%uint8 climb_pwm~%~%# system status~%uint8 sys_status~%~%# origin data~%float32 yaw_angle~%float32 pitch_angle~%float32 roll_angle~%float32 yaw_rate~%float32 pitch_rate~%float32 roll_rate~%uint32 board_time # us~%~%# tuning mode value~%uint8 pid_id~%float32 pid_ext_err # degree * 100~%float32 pid_int_err # degree / s * 10~%float32 ext_p_output~%float32 ext_i_output~%float32 ext_d_output~%float32 int_p_output~%float32 int_i_output~%float32 int_d_output~%~%~%# modes~%uint8 FAULT_MODE = 0 # 0x00~%uint8 START_MODE = 8 # 0x08~%uint8 MANUAL_MODE = 16 # 0x10~%uint8 FLIGHT_MODE = 24 # 0x18~%uint8 TUNING_MODE = 56 # 0x38~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MAVStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu_data))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'odom))
     1
     1
     1
     1
     1
     1
     4
     4
     4
     4
     4
     4
     4
     1
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MAVStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'MAVStatus
    (cl:cons ':header (header msg))
    (cl:cons ':mode_id (mode_id msg))
    (cl:cons ':imu_data (imu_data msg))
    (cl:cons ':odom (odom msg))
    (cl:cons ':mid_servo_pwm (mid_servo_pwm msg))
    (cl:cons ':left_servo_pwm (left_servo_pwm msg))
    (cl:cons ':right_servo_pwm (right_servo_pwm msg))
    (cl:cons ':throttle_pwm (throttle_pwm msg))
    (cl:cons ':climb_pwm (climb_pwm msg))
    (cl:cons ':sys_status (sys_status msg))
    (cl:cons ':yaw_angle (yaw_angle msg))
    (cl:cons ':pitch_angle (pitch_angle msg))
    (cl:cons ':roll_angle (roll_angle msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':pitch_rate (pitch_rate msg))
    (cl:cons ':roll_rate (roll_rate msg))
    (cl:cons ':board_time (board_time msg))
    (cl:cons ':pid_id (pid_id msg))
    (cl:cons ':pid_ext_err (pid_ext_err msg))
    (cl:cons ':pid_int_err (pid_int_err msg))
    (cl:cons ':ext_p_output (ext_p_output msg))
    (cl:cons ':ext_i_output (ext_i_output msg))
    (cl:cons ':ext_d_output (ext_d_output msg))
    (cl:cons ':int_p_output (int_p_output msg))
    (cl:cons ':int_i_output (int_i_output msg))
    (cl:cons ':int_d_output (int_d_output msg))
))
