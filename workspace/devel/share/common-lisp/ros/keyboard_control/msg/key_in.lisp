; Auto-generated. Do not edit!


(cl:in-package keyboard_control-msg)


;//! \htmlinclude key_in.msg.html

(cl:defclass <key_in> (roslisp-msg-protocol:ros-message)
  ((keycode
    :reader keycode
    :initarg :keycode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass key_in (<key_in>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <key_in>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'key_in)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name keyboard_control-msg:<key_in> is deprecated: use keyboard_control-msg:key_in instead.")))

(cl:ensure-generic-function 'keycode-val :lambda-list '(m))
(cl:defmethod keycode-val ((m <key_in>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard_control-msg:keycode-val is deprecated.  Use keyboard_control-msg:keycode instead.")
  (keycode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <key_in>) ostream)
  "Serializes a message object of type '<key_in>"
  (cl:let* ((signed (cl:slot-value msg 'keycode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <key_in>) istream)
  "Deserializes a message object of type '<key_in>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'keycode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<key_in>)))
  "Returns string type for a message object of type '<key_in>"
  "keyboard_control/key_in")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'key_in)))
  "Returns string type for a message object of type 'key_in"
  "keyboard_control/key_in")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<key_in>)))
  "Returns md5sum for a message object of type '<key_in>"
  "622b0edfae335c0cbebcebf432e2cce0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'key_in)))
  "Returns md5sum for a message object of type 'key_in"
  "622b0edfae335c0cbebcebf432e2cce0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<key_in>)))
  "Returns full string definition for message of type '<key_in>"
  (cl:format cl:nil "int8 keycode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'key_in)))
  "Returns full string definition for message of type 'key_in"
  (cl:format cl:nil "int8 keycode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <key_in>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <key_in>))
  "Converts a ROS message object to a list"
  (cl:list 'key_in
    (cl:cons ':keycode (keycode msg))
))
