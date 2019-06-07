; Auto-generated. Do not edit!


(cl:in-package common_msgs-msg)


;//! \htmlinclude Cone.msg.html

(cl:defclass <Cone> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Cone (<Cone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_msgs-msg:<Cone> is deprecated: use common_msgs-msg:Cone instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Cone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:position-val is deprecated.  Use common_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <Cone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:color-val is deprecated.  Use common_msgs-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Cone>)))
    "Constants for message type '<Cone>"
  '((:BLUE . 0)
    (:YELLOW . 1)
    (:ORANGE . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Cone)))
    "Constants for message type 'Cone"
  '((:BLUE . 0)
    (:YELLOW . 1)
    (:ORANGE . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cone>) ostream)
  "Serializes a message object of type '<Cone>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'color)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cone>) istream)
  "Deserializes a message object of type '<Cone>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'color)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cone>)))
  "Returns string type for a message object of type '<Cone>"
  "common_msgs/Cone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cone)))
  "Returns string type for a message object of type 'Cone"
  "common_msgs/Cone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cone>)))
  "Returns md5sum for a message object of type '<Cone>"
  "b30308676f596f91d4defb29138731c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cone)))
  "Returns md5sum for a message object of type 'Cone"
  "b30308676f596f91d4defb29138731c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cone>)))
  "Returns full string definition for message of type '<Cone>"
  (cl:format cl:nil "uint8 BLUE=0~%uint8 YELLOW=1~%uint8 ORANGE=2~%~%geometry_msgs/Point position~%uint8 color~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cone)))
  "Returns full string definition for message of type 'Cone"
  (cl:format cl:nil "uint8 BLUE=0~%uint8 YELLOW=1~%uint8 ORANGE=2~%~%geometry_msgs/Point position~%uint8 color~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cone>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cone>))
  "Converts a ROS message object to a list"
  (cl:list 'Cone
    (cl:cons ':position (position msg))
    (cl:cons ':color (color msg))
))
