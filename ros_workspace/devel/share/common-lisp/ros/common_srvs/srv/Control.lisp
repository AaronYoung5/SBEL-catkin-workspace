; Auto-generated. Do not edit!


(cl:in-package common_srvs-srv)


;//! \htmlinclude Control-request.msg.html

(cl:defclass <Control-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Control-request (<Control-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_srvs-srv:<Control-request> is deprecated: use common_srvs-srv:Control-request instead.")))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <Control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_srvs-srv:throttle-val is deprecated.  Use common_srvs-srv:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'braking-val :lambda-list '(m))
(cl:defmethod braking-val ((m <Control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_srvs-srv:braking-val is deprecated.  Use common_srvs-srv:braking instead.")
  (braking m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <Control-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_srvs-srv:steering-val is deprecated.  Use common_srvs-srv:steering instead.")
  (steering m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control-request>) ostream)
  "Serializes a message object of type '<Control-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'throttle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'braking) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'steering) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control-request>) istream)
  "Deserializes a message object of type '<Control-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'throttle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'braking) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'steering) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control-request>)))
  "Returns string type for a service object of type '<Control-request>"
  "common_srvs/ControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control-request)))
  "Returns string type for a service object of type 'Control-request"
  "common_srvs/ControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control-request>)))
  "Returns md5sum for a message object of type '<Control-request>"
  "a8a968ff18098750e2487c37c9a29dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control-request)))
  "Returns md5sum for a message object of type 'Control-request"
  "a8a968ff18098750e2487c37c9a29dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control-request>)))
  "Returns full string definition for message of type '<Control-request>"
  (cl:format cl:nil "std_msgs/Float32 throttle~%std_msgs/Float32 braking~%std_msgs/Float32 steering~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control-request)))
  "Returns full string definition for message of type 'Control-request"
  (cl:format cl:nil "std_msgs/Float32 throttle~%std_msgs/Float32 braking~%std_msgs/Float32 steering~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'throttle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'braking))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'steering))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Control-request
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':braking (braking msg))
    (cl:cons ':steering (steering msg))
))
;//! \htmlinclude Control-response.msg.html

(cl:defclass <Control-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Control-response (<Control-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_srvs-srv:<Control-response> is deprecated: use common_srvs-srv:Control-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control-response>) ostream)
  "Serializes a message object of type '<Control-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control-response>) istream)
  "Deserializes a message object of type '<Control-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control-response>)))
  "Returns string type for a service object of type '<Control-response>"
  "common_srvs/ControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control-response)))
  "Returns string type for a service object of type 'Control-response"
  "common_srvs/ControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control-response>)))
  "Returns md5sum for a message object of type '<Control-response>"
  "a8a968ff18098750e2487c37c9a29dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control-response)))
  "Returns md5sum for a message object of type 'Control-response"
  "a8a968ff18098750e2487c37c9a29dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control-response>)))
  "Returns full string definition for message of type '<Control-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control-response)))
  "Returns full string definition for message of type 'Control-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Control-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Control)))
  'Control-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Control)))
  'Control-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control)))
  "Returns string type for a service object of type '<Control>"
  "common_srvs/Control")