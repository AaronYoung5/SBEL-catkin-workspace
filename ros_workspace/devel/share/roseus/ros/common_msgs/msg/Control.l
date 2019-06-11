;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::Control)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'Control (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::CONTROL")
  (make-package "COMMON_MSGS::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common_msgs::Control
  :super ros::object
  :slots (_throttle _braking _steering ))

(defmethod common_msgs::Control
  (:init
   (&key
    ((:throttle __throttle) (instance std_msgs::Float32 :init))
    ((:braking __braking) (instance std_msgs::Float32 :init))
    ((:steering __steering) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _throttle __throttle)
   (setq _braking __braking)
   (setq _steering __steering)
   self)
  (:throttle
   (&rest __throttle)
   (if (keywordp (car __throttle))
       (send* _throttle __throttle)
     (progn
       (if __throttle (setq _throttle (car __throttle)))
       _throttle)))
  (:braking
   (&rest __braking)
   (if (keywordp (car __braking))
       (send* _braking __braking)
     (progn
       (if __braking (setq _braking (car __braking)))
       _braking)))
  (:steering
   (&rest __steering)
   (if (keywordp (car __steering))
       (send* _steering __steering)
     (progn
       (if __steering (setq _steering (car __steering)))
       _steering)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32 _throttle
    (send _throttle :serialization-length)
    ;; std_msgs/Float32 _braking
    (send _braking :serialization-length)
    ;; std_msgs/Float32 _steering
    (send _steering :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32 _throttle
       (send _throttle :serialize s)
     ;; std_msgs/Float32 _braking
       (send _braking :serialize s)
     ;; std_msgs/Float32 _steering
       (send _steering :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32 _throttle
     (send _throttle :deserialize buf ptr-) (incf ptr- (send _throttle :serialization-length))
   ;; std_msgs/Float32 _braking
     (send _braking :deserialize buf ptr-) (incf ptr- (send _braking :serialization-length))
   ;; std_msgs/Float32 _steering
     (send _steering :deserialize buf ptr-) (incf ptr- (send _steering :serialization-length))
   ;;
   self)
  )

(setf (get common_msgs::Control :md5sum-) "a8a968ff18098750e2487c37c9a29dba")
(setf (get common_msgs::Control :datatype-) "common_msgs/Control")
(setf (get common_msgs::Control :definition-)
      "std_msgs/Float32 throttle
std_msgs/Float32 braking
std_msgs/Float32 steering

================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :common_msgs/Control "a8a968ff18098750e2487c37c9a29dba")


