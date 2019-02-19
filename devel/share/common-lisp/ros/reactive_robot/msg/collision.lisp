; Auto-generated. Do not edit!


(cl:in-package reactive_robot-msg)


;//! \htmlinclude collision.msg.html

(cl:defclass <collision> (roslisp-msg-protocol:ros-message)
  ((collision
    :reader collision
    :initarg :collision
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass collision (<collision>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <collision>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'collision)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reactive_robot-msg:<collision> is deprecated: use reactive_robot-msg:collision instead.")))

(cl:ensure-generic-function 'collision-val :lambda-list '(m))
(cl:defmethod collision-val ((m <collision>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reactive_robot-msg:collision-val is deprecated.  Use reactive_robot-msg:collision instead.")
  (collision m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <collision>) ostream)
  "Serializes a message object of type '<collision>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'collision) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <collision>) istream)
  "Deserializes a message object of type '<collision>"
    (cl:setf (cl:slot-value msg 'collision) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<collision>)))
  "Returns string type for a message object of type '<collision>"
  "reactive_robot/collision")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'collision)))
  "Returns string type for a message object of type 'collision"
  "reactive_robot/collision")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<collision>)))
  "Returns md5sum for a message object of type '<collision>"
  "ec9653804a13642f770edbe4a85843b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'collision)))
  "Returns md5sum for a message object of type 'collision"
  "ec9653804a13642f770edbe4a85843b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<collision>)))
  "Returns full string definition for message of type '<collision>"
  (cl:format cl:nil "bool collision~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'collision)))
  "Returns full string definition for message of type 'collision"
  (cl:format cl:nil "bool collision~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <collision>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <collision>))
  "Converts a ROS message object to a list"
  (cl:list 'collision
    (cl:cons ':collision (collision msg))
))
