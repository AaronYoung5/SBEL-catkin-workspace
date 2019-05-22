;; Auto-generated. Do not edit!


(when (boundp 'keyboard_control::key_in)
  (if (not (find-package "KEYBOARD_CONTROL"))
    (make-package "KEYBOARD_CONTROL"))
  (shadow 'key_in (find-package "KEYBOARD_CONTROL")))
(unless (find-package "KEYBOARD_CONTROL::KEY_IN")
  (make-package "KEYBOARD_CONTROL::KEY_IN"))

(in-package "ROS")
;;//! \htmlinclude key_in.msg.html


(defclass keyboard_control::key_in
  :super ros::object
  :slots (_keycode ))

(defmethod keyboard_control::key_in
  (:init
   (&key
    ((:keycode __keycode) 0)
    )
   (send-super :init)
   (setq _keycode (round __keycode))
   self)
  (:keycode
   (&optional __keycode)
   (if __keycode (setq _keycode __keycode)) _keycode)
  (:serialization-length
   ()
   (+
    ;; int8 _keycode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _keycode
       (write-byte _keycode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _keycode
     (setq _keycode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _keycode 127) (setq _keycode (- _keycode 256)))
   ;;
   self)
  )

(setf (get keyboard_control::key_in :md5sum-) "622b0edfae335c0cbebcebf432e2cce0")
(setf (get keyboard_control::key_in :datatype-) "keyboard_control/key_in")
(setf (get keyboard_control::key_in :definition-)
      "int8 keycode

")



(provide :keyboard_control/key_in "622b0edfae335c0cbebcebf432e2cce0")


