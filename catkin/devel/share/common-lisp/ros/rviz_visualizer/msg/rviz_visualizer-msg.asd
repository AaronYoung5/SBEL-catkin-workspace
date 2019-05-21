
(cl:in-package :asdf)

(defsystem "rviz_visualizer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "path_msg" :depends-on ("_package_path_msg"))
    (:file "_package_path_msg" :depends-on ("_package"))
  ))