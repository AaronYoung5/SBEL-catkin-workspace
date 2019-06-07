
(cl:in-package :asdf)

(defsystem "chrono_ros_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Cones" :depends-on ("_package_Cones"))
    (:file "_package_Cones" :depends-on ("_package"))
  ))