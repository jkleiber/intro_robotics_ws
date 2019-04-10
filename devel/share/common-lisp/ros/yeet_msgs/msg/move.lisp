; Auto-generated. Do not edit!


(cl:in-package yeet_msgs-msg)


;//! \htmlinclude move.msg.html

(cl:defclass <move> (roslisp-msg-protocol:ros-message)
  ((todo
    :reader todo
    :initarg :todo
    :type cl:fixnum
    :initform 0))
)

(cl:defclass move (<move>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yeet_msgs-msg:<move> is deprecated: use yeet_msgs-msg:move instead.")))

(cl:ensure-generic-function 'todo-val :lambda-list '(m))
(cl:defmethod todo-val ((m <move>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yeet_msgs-msg:todo-val is deprecated.  Use yeet_msgs-msg:todo instead.")
  (todo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move>) ostream)
  "Serializes a message object of type '<move>"
  (cl:let* ((signed (cl:slot-value msg 'todo)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move>) istream)
  "Deserializes a message object of type '<move>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'todo) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move>)))
  "Returns string type for a message object of type '<move>"
  "yeet_msgs/move")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move)))
  "Returns string type for a message object of type 'move"
  "yeet_msgs/move")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move>)))
  "Returns md5sum for a message object of type '<move>"
  "cf55fd99d4d1be34562aaf532f8ee9a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move)))
  "Returns md5sum for a message object of type 'move"
  "cf55fd99d4d1be34562aaf532f8ee9a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move>)))
  "Returns full string definition for message of type '<move>"
  (cl:format cl:nil "int8 todo~%#test~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move)))
  "Returns full string definition for message of type 'move"
  (cl:format cl:nil "int8 todo~%#test~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move>))
  "Converts a ROS message object to a list"
  (cl:list 'move
    (cl:cons ':todo (todo msg))
))
