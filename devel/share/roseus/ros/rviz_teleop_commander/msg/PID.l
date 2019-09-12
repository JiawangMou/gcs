;; Auto-generated. Do not edit!


(when (boundp 'rviz_teleop_commander::PID)
  (if (not (find-package "RVIZ_TELEOP_COMMANDER"))
    (make-package "RVIZ_TELEOP_COMMANDER"))
  (shadow 'PID (find-package "RVIZ_TELEOP_COMMANDER")))
(unless (find-package "RVIZ_TELEOP_COMMANDER::PID")
  (make-package "RVIZ_TELEOP_COMMANDER::PID"))

(in-package "ROS")
;;//! \htmlinclude PID.msg.html


(defclass rviz_teleop_commander::PID
  :super ros::object
  :slots (_p_value _i_value _d_value ))

(defmethod rviz_teleop_commander::PID
  (:init
   (&key
    ((:p_value __p_value) 0.0)
    ((:i_value __i_value) 0.0)
    ((:d_value __d_value) 0.0)
    )
   (send-super :init)
   (setq _p_value (float __p_value))
   (setq _i_value (float __i_value))
   (setq _d_value (float __d_value))
   self)
  (:p_value
   (&optional __p_value)
   (if __p_value (setq _p_value __p_value)) _p_value)
  (:i_value
   (&optional __i_value)
   (if __i_value (setq _i_value __i_value)) _i_value)
  (:d_value
   (&optional __d_value)
   (if __d_value (setq _d_value __d_value)) _d_value)
  (:serialization-length
   ()
   (+
    ;; float32 _p_value
    4
    ;; float32 _i_value
    4
    ;; float32 _d_value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _p_value
       (sys::poke _p_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _i_value
       (sys::poke _i_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d_value
       (sys::poke _d_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _p_value
     (setq _p_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _i_value
     (setq _i_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d_value
     (setq _d_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rviz_teleop_commander::PID :md5sum-) "adb7a9d75543c075dfce10e2d06a07d9")
(setf (get rviz_teleop_commander::PID :datatype-) "rviz_teleop_commander/PID")
(setf (get rviz_teleop_commander::PID :definition-)
      "float32 p_value
float32 i_value
float32 d_value
")



(provide :rviz_teleop_commander/PID "adb7a9d75543c075dfce10e2d06a07d9")


