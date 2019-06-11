; Auto-generated. Do not edit!


(cl:in-package common_msgs-msg)


;//! \htmlinclude Control.msg.html

(cl:defclass <Control> (roslisp-msg-protocol:ros-message)
  ((throttle
    :reader throttle
    :initarg :throttle
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (braking
    :reader braking
    :initarg :braking
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (steering
    :reader steering
    :initarg :steering
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass Control (<Control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_msgs-msg:<Control> is deprecated: use common_msgs-msg:Control instead.")))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:throttle-val is deprecated.  Use common_msgs-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'braking-val :lambda-list '(m))
(cl:defmethod braking-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:braking-val is deprecated.  Use common_msgs-msg:braking instead.")
  (braking m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:steering-val is deprecated.  Use common_msgs-msg:steering instead.")
  (steering m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control>) ostream)
  "Serializes a message object of type '<Control>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'throttle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'braking) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'steering) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control>) istream)
  "Deserializes a message object of type '<Control>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'throttle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'braking) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'steering) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control>)))
  "Returns string type for a message object of type '<Control>"
  "common_msgs/Control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control)))
  "Returns string type for a message object of type 'Control"
  "common_msgs/Control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control>)))
  "Returns md5sum for a message object of type '<Control>"
  "a8a968ff18098750e2487c37c9a29dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control)))
  "Returns md5sum for a message object of type 'Control"
  "a8a968ff18098750e2487c37c9a29dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control>)))
  "Returns full string definition for message of type '<Control>"
  (cl:format cl:nil "std_msgs/Float32 throttle~%std_msgs/Float32 braking~%std_msgs/Float32 steering~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control)))
  "Returns full string definition for message of type 'Control"
  (cl:format cl:nil "std_msgs/Float32 throttle~%std_msgs/Float32 braking~%std_msgs/Float32 steering~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'throttle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'braking))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'steering))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control>))
  "Converts a ROS message object to a list"
  (cl:list 'Control
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':braking (braking msg))
    (cl:cons ':steering (steering msg))
))
