; Auto-generated. Do not edit!


(cl:in-package common_msgs-msg)


;//! \htmlinclude ConeMap.msg.html

(cl:defclass <ConeMap> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (blue_cones
    :reader blue_cones
    :initarg :blue_cones
    :type (cl:vector common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'common_msgs-msg:Cone :initial-element (cl:make-instance 'common_msgs-msg:Cone)))
   (yellow_cones
    :reader yellow_cones
    :initarg :yellow_cones
    :type (cl:vector common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'common_msgs-msg:Cone :initial-element (cl:make-instance 'common_msgs-msg:Cone)))
   (orange_cones
    :reader orange_cones
    :initarg :orange_cones
    :type (cl:vector common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'common_msgs-msg:Cone :initial-element (cl:make-instance 'common_msgs-msg:Cone))))
)

(cl:defclass ConeMap (<ConeMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConeMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConeMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_msgs-msg:<ConeMap> is deprecated: use common_msgs-msg:ConeMap instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:header-val is deprecated.  Use common_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'blue_cones-val :lambda-list '(m))
(cl:defmethod blue_cones-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:blue_cones-val is deprecated.  Use common_msgs-msg:blue_cones instead.")
  (blue_cones m))

(cl:ensure-generic-function 'yellow_cones-val :lambda-list '(m))
(cl:defmethod yellow_cones-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:yellow_cones-val is deprecated.  Use common_msgs-msg:yellow_cones instead.")
  (yellow_cones m))

(cl:ensure-generic-function 'orange_cones-val :lambda-list '(m))
(cl:defmethod orange_cones-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:orange_cones-val is deprecated.  Use common_msgs-msg:orange_cones instead.")
  (orange_cones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConeMap>) ostream)
  "Serializes a message object of type '<ConeMap>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'orange_cones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'orange_cones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConeMap>) istream)
  "Deserializes a message object of type '<ConeMap>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blue_cones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blue_cones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yellow_cones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'yellow_cones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'orange_cones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'orange_cones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConeMap>)))
  "Returns string type for a message object of type '<ConeMap>"
  "common_msgs/ConeMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConeMap)))
  "Returns string type for a message object of type 'ConeMap"
  "common_msgs/ConeMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConeMap>)))
  "Returns md5sum for a message object of type '<ConeMap>"
  "9a5312a83690b1beb889fc4f459cf44b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConeMap)))
  "Returns md5sum for a message object of type 'ConeMap"
  "9a5312a83690b1beb889fc4f459cf44b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConeMap>)))
  "Returns full string definition for message of type '<ConeMap>"
  (cl:format cl:nil "std_msgs/Header header~%~%common_msgs/Cone[] blue_cones~%common_msgs/Cone[] yellow_cones~%common_msgs/Cone[] orange_cones~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: common_msgs/Cone~%uint8 BLUE=0~%uint8 YELLOW=1~%uint8 ORANGE=2~%~%geometry_msgs/Point position~%uint8 color~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConeMap)))
  "Returns full string definition for message of type 'ConeMap"
  (cl:format cl:nil "std_msgs/Header header~%~%common_msgs/Cone[] blue_cones~%common_msgs/Cone[] yellow_cones~%common_msgs/Cone[] orange_cones~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: common_msgs/Cone~%uint8 BLUE=0~%uint8 YELLOW=1~%uint8 ORANGE=2~%~%geometry_msgs/Point position~%uint8 color~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConeMap>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blue_cones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'yellow_cones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'orange_cones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConeMap>))
  "Converts a ROS message object to a list"
  (cl:list 'ConeMap
    (cl:cons ':header (header msg))
    (cl:cons ':blue_cones (blue_cones msg))
    (cl:cons ':yellow_cones (yellow_cones msg))
    (cl:cons ':orange_cones (orange_cones msg))
))
