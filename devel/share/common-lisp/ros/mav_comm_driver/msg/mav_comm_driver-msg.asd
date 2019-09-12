
(cl:in-package :asdf)

(defsystem "mav_comm_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MAVStatus" :depends-on ("_package_MAVStatus"))
    (:file "_package_MAVStatus" :depends-on ("_package"))
    (:file "ModeConfig" :depends-on ("_package_ModeConfig"))
    (:file "_package_ModeConfig" :depends-on ("_package"))
  ))