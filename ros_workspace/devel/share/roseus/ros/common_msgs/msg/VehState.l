;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::VehState)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'VehState (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::VEHSTATE")
  (make-package "COMMON_MSGS::VEHSTATE"))

(in-package "ROS")
;;//! \htmlinclude VehState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common_msgs::VehState
  :super ros::object
  :slots (_header _state _state_dt ))

(defmethod common_msgs::VehState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) (instance geometry_msgs::Pose :init))
    ((:state_dt __state_dt) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state __state)
   (setq _state_dt __state_dt)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:state_dt
   (&rest __state_dt)
   (if (keywordp (car __state_dt))
       (send* _state_dt __state_dt)
     (progn
       (if __state_dt (setq _state_dt (car __state_dt)))
       _state_dt)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _state
    (send _state :serialization-length)
    ;; geometry_msgs/Pose _state_dt
    (send _state_dt :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _state
       (send _state :serialize s)
     ;; geometry_msgs/Pose _state_dt
       (send _state_dt :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;; geometry_msgs/Pose _state_dt
     (send _state_dt :deserialize buf ptr-) (incf ptr- (send _state_dt :serialization-length))
   ;;
   self)
  )

(setf (get common_msgs::VehState :md5sum-) "ed712ac62de968ce94aaf5e2f998496b")
(setf (get common_msgs::VehState :datatype-) "common_msgs/VehState")
(setf (get common_msgs::VehState :definition-)
      "std_msgs/Header header

geometry_msgs/Pose state    # position orientation
geometry_msgs/Pose state_dt # position orientation

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :common_msgs/VehState "ed712ac62de968ce94aaf5e2f998496b")


