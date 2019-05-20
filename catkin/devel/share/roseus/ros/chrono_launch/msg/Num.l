;; Auto-generated. Do not edit!


(when (boundp 'chrono_launch::Num)
  (if (not (find-package "CHRONO_LAUNCH"))
    (make-package "CHRONO_LAUNCH"))
  (shadow 'Num (find-package "CHRONO_LAUNCH")))
(unless (find-package "CHRONO_LAUNCH::NUM")
  (make-package "CHRONO_LAUNCH::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass chrono_launch::Num
  :super ros::object
  :slots ())

(defmethod chrono_launch::Num
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

(setf (get chrono_launch::Num :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get chrono_launch::Num :datatype-) "chrono_launch/Num")
(setf (get chrono_launch::Num :definition-)
      "
")



(provide :chrono_launch/Num "d41d8cd98f00b204e9800998ecf8427e")


