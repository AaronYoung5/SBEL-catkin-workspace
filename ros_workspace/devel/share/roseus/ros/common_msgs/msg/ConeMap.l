;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::ConeMap)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'ConeMap (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::CONEMAP")
  (make-package "COMMON_MSGS::CONEMAP"))

(in-package "ROS")
;;//! \htmlinclude ConeMap.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common_msgs::ConeMap
  :super ros::object
  :slots (_header _blue_cones _yellow_cones _orange_cones ))

(defmethod common_msgs::ConeMap
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:blue_cones __blue_cones) (let (r) (dotimes (i 0) (push (instance common_msgs::Cone :init) r)) r))
    ((:yellow_cones __yellow_cones) (let (r) (dotimes (i 0) (push (instance common_msgs::Cone :init) r)) r))
    ((:orange_cones __orange_cones) (let (r) (dotimes (i 0) (push (instance common_msgs::Cone :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _blue_cones __blue_cones)
   (setq _yellow_cones __yellow_cones)
   (setq _orange_cones __orange_cones)
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
  (:orange_cones
   (&rest __orange_cones)
   (if (keywordp (car __orange_cones))
       (send* _orange_cones __orange_cones)
     (progn
       (if __orange_cones (setq _orange_cones (car __orange_cones)))
       _orange_cones)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; common_msgs/Cone[] _blue_cones
    (apply #'+ (send-all _blue_cones :serialization-length)) 4
    ;; common_msgs/Cone[] _yellow_cones
    (apply #'+ (send-all _yellow_cones :serialization-length)) 4
    ;; common_msgs/Cone[] _orange_cones
    (apply #'+ (send-all _orange_cones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; common_msgs/Cone[] _blue_cones
     (write-long (length _blue_cones) s)
     (dolist (elem _blue_cones)
       (send elem :serialize s)
       )
     ;; common_msgs/Cone[] _yellow_cones
     (write-long (length _yellow_cones) s)
     (dolist (elem _yellow_cones)
       (send elem :serialize s)
       )
     ;; common_msgs/Cone[] _orange_cones
     (write-long (length _orange_cones) s)
     (dolist (elem _orange_cones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; common_msgs/Cone[] _blue_cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _blue_cones (let (r) (dotimes (i n) (push (instance common_msgs::Cone :init) r)) r))
     (dolist (elem- _blue_cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; common_msgs/Cone[] _yellow_cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yellow_cones (let (r) (dotimes (i n) (push (instance common_msgs::Cone :init) r)) r))
     (dolist (elem- _yellow_cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; common_msgs/Cone[] _orange_cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _orange_cones (let (r) (dotimes (i n) (push (instance common_msgs::Cone :init) r)) r))
     (dolist (elem- _orange_cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get common_msgs::ConeMap :md5sum-) "9a5312a83690b1beb889fc4f459cf44b")
(setf (get common_msgs::ConeMap :datatype-) "common_msgs/ConeMap")
(setf (get common_msgs::ConeMap :definition-)
      "std_msgs/Header header

common_msgs/Cone[] blue_cones
common_msgs/Cone[] yellow_cones
common_msgs/Cone[] orange_cones

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
MSG: common_msgs/Cone
uint8 BLUE=0
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



(provide :common_msgs/ConeMap "9a5312a83690b1beb889fc4f459cf44b")


