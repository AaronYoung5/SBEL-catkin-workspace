; Auto-generated. Do not edit!


(cl:in-package common_msgs-msg)


;//! \htmlinclude VehState.msg.html

(cl:defclass <VehState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (state_dt
    :reader state_dt
    :initarg :state_dt
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass VehState (<VehState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_msgs-msg:<VehState> is deprecated: use common_msgs-msg:VehState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:header-val is deprecated.  Use common_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <VehState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:state-val is deprecated.  Use common_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'state_dt-val :lambda-list '(m))
(cl:defmethod state_dt-val ((m <VehState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:state_dt-val is deprecated.  Use common_msgs-msg:state_dt instead.")
  (state_dt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehState>) ostream)
  "Serializes a message object of type '<VehState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state_dt) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehState>) istream)
  "Deserializes a message object of type '<VehState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state_dt) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehState>)))
  "Returns string type for a message object of type '<VehState>"
  "common_msgs/VehState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehState)))
  "Returns string type for a message object of type 'VehState"
  "common_msgs/VehState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehState>)))
  "Returns md5sum for a message object of type '<VehState>"
  "ed712ac62de968ce94aaf5e2f998496b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehState)))
  "Returns md5sum for a message object of type 'VehState"
  "ed712ac62de968ce94aaf5e2f998496b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehState>)))
  "Returns full string definition for message of type '<VehState>"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Pose state    # position orientation~%geometry_msgs/Pose state_dt # position orientation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehState)))
  "Returns full string definition for message of type 'VehState"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Pose state    # position orientation~%geometry_msgs/Pose state_dt # position orientation~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state_dt))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehState>))
  "Converts a ROS message object to a list"
  (cl:list 'VehState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':state_dt (state_dt msg))
))
