; Auto-generated. Do not edit!


(cl:in-package yeet_msgs-msg)


;//! \htmlinclude keyboard.msg.html

(cl:defclass <keyboard> (roslisp-msg-protocol:ros-message)
  ((c
    :reader c
    :initarg :c
    :type cl:integer
    :initform 0))
)

(cl:defclass keyboard (<keyboard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyboard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyboard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yeet_msgs-msg:<keyboard> is deprecated: use yeet_msgs-msg:keyboard instead.")))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yeet_msgs-msg:c-val is deprecated.  Use yeet_msgs-msg:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyboard>) ostream)
  "Serializes a message object of type '<keyboard>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'c)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyboard>) istream)
  "Deserializes a message object of type '<keyboard>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'c)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyboard>)))
  "Returns string type for a message object of type '<keyboard>"
  "yeet_msgs/keyboard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard)))
  "Returns string type for a message object of type 'keyboard"
  "yeet_msgs/keyboard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyboard>)))
  "Returns md5sum for a message object of type '<keyboard>"
  "503f37a585b485611c99195decce8bba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyboard)))
  "Returns md5sum for a message object of type 'keyboard"
  "503f37a585b485611c99195decce8bba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyboard>)))
  "Returns full string definition for message of type '<keyboard>"
  (cl:format cl:nil "char c~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyboard)))
  "Returns full string definition for message of type 'keyboard"
  (cl:format cl:nil "char c~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyboard>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyboard>))
  "Converts a ROS message object to a list"
  (cl:list 'keyboard
    (cl:cons ':c (c msg))
))
