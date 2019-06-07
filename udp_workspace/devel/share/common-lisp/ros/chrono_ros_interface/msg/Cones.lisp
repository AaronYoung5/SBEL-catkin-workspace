; Auto-generated. Do not edit!


(cl:in-package chrono_ros_interface-msg)


;//! \htmlinclude Cones.msg.html

(cl:defclass <Cones> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (blue_cones
    :reader blue_cones
    :initarg :blue_cones
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (yellow_cones
    :reader yellow_cones
    :initarg :yellow_cones
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (size
    :reader size
    :initarg :size
    :type std_msgs-msg:Int16
    :initform (cl:make-instance 'std_msgs-msg:Int16)))
)

(cl:defclass Cones (<Cones>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cones>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cones)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chrono_ros_interface-msg:<Cones> is deprecated: use chrono_ros_interface-msg:Cones instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Cones>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chrono_ros_interface-msg:header-val is deprecated.  Use chrono_ros_interface-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'blue_cones-val :lambda-list '(m))
(cl:defmethod blue_cones-val ((m <Cones>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chrono_ros_interface-msg:blue_cones-val is deprecated.  Use chrono_ros_interface-msg:blue_cones instead.")
  (blue_cones m))

(cl:ensure-generic-function 'yellow_cones-val :lambda-list '(m))
(cl:defmethod yellow_cones-val ((m <Cones>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chrono_ros_interface-msg:yellow_cones-val is deprecated.  Use chrono_ros_interface-msg:yellow_cones instead.")
  (yellow_cones m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <Cones>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chrono_ros_interface-msg:size-val is deprecated.  Use chrono_ros_interface-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cones>) ostream)
  "Serializes a message object of type '<Cones>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blue_cones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'blue_cones))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'yellow_cones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'yellow_cones))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cones>) istream)
  "Deserializes a message object of type '<Cones>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blue_cones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blue_cones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yellow_cones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'yellow_cones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cones>)))
  "Returns string type for a message object of type '<Cones>"
  "chrono_ros_interface/Cones")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cones)))
  "Returns string type for a message object of type 'Cones"
  "chrono_ros_interface/Cones")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cones>)))
  "Returns md5sum for a message object of type '<Cones>"
  "66302a9cec1053e5e1d2dec8fe6c53d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cones)))
  "Returns md5sum for a message object of type 'Cones"
  "66302a9cec1053e5e1d2dec8fe6c53d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cones>)))
  "Returns full string definition for message of type '<Cones>"
  (cl:format cl:nil "# Similar to nav_msgs/Path.msg, but has two paths (blue and yellow)~%# Positions are relative to the vehicle, not global~%Header header~%geometry_msgs/PoseStamped[] blue_cones~%geometry_msgs/PoseStamped[] yellow_cones~%std_msgs/Int16 size~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Int16~%int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cones)))
  "Returns full string definition for message of type 'Cones"
  (cl:format cl:nil "# Similar to nav_msgs/Path.msg, but has two paths (blue and yellow)~%# Positions are relative to the vehicle, not global~%Header header~%geometry_msgs/PoseStamped[] blue_cones~%geometry_msgs/PoseStamped[] yellow_cones~%std_msgs/Int16 size~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Int16~%int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cones>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blue_cones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'yellow_cones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cones>))
  "Converts a ROS message object to a list"
  (cl:list 'Cones
    (cl:cons ':header (header msg))
    (cl:cons ':blue_cones (blue_cones msg))
    (cl:cons ':yellow_cones (yellow_cones msg))
    (cl:cons ':size (size msg))
))
