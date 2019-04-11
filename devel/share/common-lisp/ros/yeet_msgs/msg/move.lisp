; Auto-generated. Do not edit!


(cl:in-package yeet_msgs-msg)


;//! \htmlinclude move.msg.html

(cl:defclass <move> (roslisp-msg-protocol:ros-message)
  ((drive
    :reader drive
    :initarg :drive
    :type cl:fixnum
    :initform 0)
   (turn
    :reader turn
    :initarg :turn
    :type cl:fixnum
    :initform 0))
)

(cl:defclass move (<move>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yeet_msgs-msg:<move> is deprecated: use yeet_msgs-msg:move instead.")))

(cl:ensure-generic-function 'drive-val :lambda-list '(m))
(cl:defmethod drive-val ((m <move>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yeet_msgs-msg:drive-val is deprecated.  Use yeet_msgs-msg:drive instead.")
  (drive m))

(cl:ensure-generic-function 'turn-val :lambda-list '(m))
(cl:defmethod turn-val ((m <move>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yeet_msgs-msg:turn-val is deprecated.  Use yeet_msgs-msg:turn instead.")
  (turn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move>) ostream)
  "Serializes a message object of type '<move>"
  (cl:let* ((signed (cl:slot-value msg 'drive)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'turn)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move>) istream)
  "Deserializes a message object of type '<move>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drive) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'turn) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
  "8f1cddb8ef3caea21484489b8f1096b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move)))
  "Returns md5sum for a message object of type 'move"
  "8f1cddb8ef3caea21484489b8f1096b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move>)))
  "Returns full string definition for message of type '<move>"
  (cl:format cl:nil "int8 drive~%int8 turn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move)))
  "Returns full string definition for message of type 'move"
  (cl:format cl:nil "int8 drive~%int8 turn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move>))
  "Converts a ROS message object to a list"
  (cl:list 'move
    (cl:cons ':drive (drive msg))
    (cl:cons ':turn (turn msg))
))
