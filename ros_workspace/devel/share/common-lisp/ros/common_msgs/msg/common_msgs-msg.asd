
(cl:in-package :asdf)

(defsystem "common_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Cone" :depends-on ("_package_Cone"))
    (:file "_package_Cone" :depends-on ("_package"))
    (:file "ConeMap" :depends-on ("_package_ConeMap"))
    (:file "_package_ConeMap" :depends-on ("_package"))
  ))