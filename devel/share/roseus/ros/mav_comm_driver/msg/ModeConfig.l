;; Auto-generated. Do not edit!


(when (boundp 'mav_comm_driver::ModeConfig)
  (if (not (find-package "MAV_COMM_DRIVER"))
    (make-package "MAV_COMM_DRIVER"))
  (shadow 'ModeConfig (find-package "MAV_COMM_DRIVER")))
(unless (find-package "MAV_COMM_DRIVER::MODECONFIG")
  (make-package "MAV_COMM_DRIVER::MODECONFIG"))

(in-package "ROS")
;;//! \htmlinclude ModeConfig.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*FAULT_MODE*" (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(shadow '*FAULT_MODE* (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(defconstant mav_comm_driver::ModeConfig::*FAULT_MODE* 0)
(intern "*START_MODE*" (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(shadow '*START_MODE* (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(defconstant mav_comm_driver::ModeConfig::*START_MODE* 8)
(intern "*MANUAL_MODE*" (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(shadow '*MANUAL_MODE* (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(defconstant mav_comm_driver::ModeConfig::*MANUAL_MODE* 16)
(intern "*FLIGHT_MODE*" (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(shadow '*FLIGHT_MODE* (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(defconstant mav_comm_driver::ModeConfig::*FLIGHT_MODE* 24)
(intern "*TUNING_MODE*" (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(shadow '*TUNING_MODE* (find-package "MAV_COMM_DRIVER::MODECONFIG"))
(defconstant mav_comm_driver::ModeConfig::*TUNING_MODE* 56)
(defclass mav_comm_driver::ModeConfig
  :super ros::object
  :slots (_header _mode_id _data ))

(defmethod mav_comm_driver::ModeConfig
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode_id __mode_id) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode_id (round __mode_id))
   (setq _data __data)
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
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _mode_id
    1
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _mode_id
       (write-byte _mode_id s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _mode_id
     (setq _mode_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get mav_comm_driver::ModeConfig :md5sum-) "5ff71842ef7f25d484ca1c2f6bb8cd74")
(setf (get mav_comm_driver::ModeConfig :datatype-) "mav_comm_driver/ModeConfig")
(setf (get mav_comm_driver::ModeConfig :definition-)
      "Header header
uint8 mode_id
uint8[] data    # must include id as header

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

")



(provide :mav_comm_driver/ModeConfig "5ff71842ef7f25d484ca1c2f6bb8cd74")


