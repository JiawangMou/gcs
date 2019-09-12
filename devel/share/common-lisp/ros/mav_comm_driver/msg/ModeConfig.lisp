; Auto-generated. Do not edit!


(cl:in-package mav_comm_driver-msg)


;//! \htmlinclude ModeConfig.msg.html

(cl:defclass <ModeConfig> (roslisp-msg-protocol:ros-message)
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
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ModeConfig (<ModeConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mav_comm_driver-msg:<ModeConfig> is deprecated: use mav_comm_driver-msg:ModeConfig instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ModeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:header-val is deprecated.  Use mav_comm_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode_id-val :lambda-list '(m))
(cl:defmethod mode_id-val ((m <ModeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:mode_id-val is deprecated.  Use mav_comm_driver-msg:mode_id instead.")
  (mode_id m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ModeConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mav_comm_driver-msg:data-val is deprecated.  Use mav_comm_driver-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ModeConfig>)))
    "Constants for message type '<ModeConfig>"
  '((:FAULT_MODE . 0)
    (:START_MODE . 8)
    (:MANUAL_MODE . 16)
    (:FLIGHT_MODE . 24)
    (:TUNING_MODE . 56))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ModeConfig)))
    "Constants for message type 'ModeConfig"
  '((:FAULT_MODE . 0)
    (:START_MODE . 8)
    (:MANUAL_MODE . 16)
    (:FLIGHT_MODE . 24)
    (:TUNING_MODE . 56))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeConfig>) ostream)
  "Serializes a message object of type '<ModeConfig>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeConfig>) istream)
  "Deserializes a message object of type '<ModeConfig>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeConfig>)))
  "Returns string type for a message object of type '<ModeConfig>"
  "mav_comm_driver/ModeConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeConfig)))
  "Returns string type for a message object of type 'ModeConfig"
  "mav_comm_driver/ModeConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeConfig>)))
  "Returns md5sum for a message object of type '<ModeConfig>"
  "5ff71842ef7f25d484ca1c2f6bb8cd74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeConfig)))
  "Returns md5sum for a message object of type 'ModeConfig"
  "5ff71842ef7f25d484ca1c2f6bb8cd74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeConfig>)))
  "Returns full string definition for message of type '<ModeConfig>"
  (cl:format cl:nil "Header header~%uint8 mode_id~%uint8[] data    # must include id as header~%~%# modes~%uint8 FAULT_MODE = 0 # 0x00~%uint8 START_MODE = 8 # 0x08~%uint8 MANUAL_MODE = 16 # 0x10~%uint8 FLIGHT_MODE = 24 # 0x18~%uint8 TUNING_MODE = 56 # 0x38~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeConfig)))
  "Returns full string definition for message of type 'ModeConfig"
  (cl:format cl:nil "Header header~%uint8 mode_id~%uint8[] data    # must include id as header~%~%# modes~%uint8 FAULT_MODE = 0 # 0x00~%uint8 START_MODE = 8 # 0x08~%uint8 MANUAL_MODE = 16 # 0x10~%uint8 FLIGHT_MODE = 24 # 0x18~%uint8 TUNING_MODE = 56 # 0x38~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeConfig>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeConfig
    (cl:cons ':header (header msg))
    (cl:cons ':mode_id (mode_id msg))
    (cl:cons ':data (data msg))
))
