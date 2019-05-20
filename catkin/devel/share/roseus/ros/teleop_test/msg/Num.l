;; Auto-generated. Do not edit!


(when (boundp 'teleop_test::Num)
  (if (not (find-package "TELEOP_TEST"))
    (make-package "TELEOP_TEST"))
  (shadow 'Num (find-package "TELEOP_TEST")))
(unless (find-package "TELEOP_TEST::NUM")
  (make-package "TELEOP_TEST::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass teleop_test::Num
  :super ros::object
  :slots ())

(defmethod teleop_test::Num
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get teleop_test::Num :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get teleop_test::Num :datatype-) "teleop_test/Num")
(setf (get teleop_test::Num :definition-)
      "
")



(provide :teleop_test/Num "d41d8cd98f00b204e9800998ecf8427e")


