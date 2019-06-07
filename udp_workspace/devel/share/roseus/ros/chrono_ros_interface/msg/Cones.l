;; Auto-generated. Do not edit!


(when (boundp 'chrono_ros_interface::Cones)
  (if (not (find-package "CHRONO_ROS_INTERFACE"))
    (make-package "CHRONO_ROS_INTERFACE"))
  (shadow 'Cones (find-package "CHRONO_ROS_INTERFACE")))
(unless (find-package "CHRONO_ROS_INTERFACE::CONES")
  (make-package "CHRONO_ROS_INTERFACE::CONES"))

(in-package "ROS")
;;//! \htmlinclude Cones.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass chrono_ros_interface::Cones
  :super ros::object
  :slots (_header _blue_cones _yellow_cones _size ))

(defmethod chrono_ros_interface::Cones
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:blue_cones __blue_cones) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:yellow_cones __yellow_cones) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:size __size) (instance std_msgs::Int16 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _blue_cones __blue_cones)
   (setq _yellow_cones __yellow_cones)
   (setq _size __size)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:blue_cones
   (&rest __blue_cones)
   (if (keywordp (car __blue_cones))
       (send* _blue_cones __blue_cones)
     (progn
       (if __blue_cones (setq _blue_cones (car __blue_cones)))
       _blue_cones)))
  (:yellow_cones
   (&rest __yellow_cones)
   (if (keywordp (car __yellow_cones))
       (send* _yellow_cones __yellow_cones)
     (progn
       (if __yellow_cones (setq _yellow_cones (car __yellow_cones)))
       _yellow_cones)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped[] _blue_cones
    (apply #'+ (send-all _blue_cones :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _yellow_cones
    (apply #'+ (send-all _yellow_cones :serialization-length)) 4
    ;; std_msgs/Int16 _size
    (send _size :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped[] _blue_cones
     (write-long (length _blue_cones) s)
     (dolist (elem _blue_cones)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _yellow_cones
     (write-long (length _yellow_cones) s)
     (dolist (elem _yellow_cones)
       (send elem :serialize s)
       )
     ;; std_msgs/Int16 _size
       (send _size :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped[] _blue_cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _blue_cones (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _blue_cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _yellow_cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yellow_cones (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _yellow_cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Int16 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;;
   self)
  )

(setf (get chrono_ros_interface::Cones :md5sum-) "66302a9cec1053e5e1d2dec8fe6c53d1")
(setf (get chrono_ros_interface::Cones :datatype-) "chrono_ros_interface/Cones")
(setf (get chrono_ros_interface::Cones :definition-)
      "# Similar to nav_msgs/Path.msg, but has two paths (blue and yellow)
# Positions are relative to the vehicle, not global
Header header
geometry_msgs/PoseStamped[] blue_cones
geometry_msgs/PoseStamped[] yellow_cones
std_msgs/Int16 size

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

================================================================================
MSG: std_msgs/Int16
int16 data

")



(provide :chrono_ros_interface/Cones "66302a9cec1053e5e1d2dec8fe6c53d1")


