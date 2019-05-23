
(cl:in-package :asdf)

(defsystem "keyboard_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "key_in" :depends-on ("_package_key_in"))
    (:file "_package_key_in" :depends-on ("_package"))
  ))