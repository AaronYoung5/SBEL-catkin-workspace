;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::Cone)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'Cone (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::CONE")
  (make-package "COMMON_MSGS::CONE"))

(in-package "ROS")
;;//! \htmlinclude Cone.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*BLUE*" (find-package "COMMON_MSGS::CONE"))
(shadow '*BLUE* (find-package "COMMON_MSGS::CONE"))
(defconstant common_msgs::Cone::*BLUE* 0)
(intern "*YELLOW*" (find-package "COMMON_MSGS::CONE"))
(shadow '*YELLOW* (find-package "COMMON_MSGS::CONE"))
(defconstant common_msgs::Cone::*YELLOW* 1)
(intern "*ORANGE*" (find-package "COMMON_MSGS::CONE"))
(shadow '*ORANGE* (find-package "COMMON_MSGS::CONE"))
(defconstant common_msgs::Cone::*ORANGE* 2)
(defclass common_msgs::Cone
  :super ros::object
  :slots (_position _color ))

(defmethod common_msgs::Cone
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:color __color) 0)
    )
   (send-super :init)
   (setq _position __position)
   (setq _color (round __color))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; uint8 _color
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; uint8 _color
       (write-byte _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; uint8 _color
     (setq _color (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get common_msgs::Cone :md5sum-) "b30308676f596f91d4defb29138731c2")
(setf (get common_msgs::Cone :datatype-) "common_msgs/Cone")
(setf (get common_msgs::Cone :definition-)
      "uint8 BLUE=0
uint8 YELLOW=1
uint8 ORANGE=2

geometry_msgs/Point position
uint8 color

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :common_msgs/Cone "b30308676f596f91d4defb29138731c2")


