; Auto-generated. Do not edit!


(cl:in-package reactive_robot-msg)


;//! \htmlinclude autodrive.msg.html

(cl:defclass <autodrive> (roslisp-msg-protocol:ros-message)
  ((collision
    :reader collision
    :initarg :collision
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass autodrive (<autodrive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <autodrive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'autodrive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reactive_robot-msg:<autodrive> is deprecated: use reactive_robot-msg:autodrive instead.")))

(cl:ensure-generic-function 'collision-val :lambda-list '(m))
(cl:defmethod collision-val ((m <autodrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reactive_robot-msg:collision-val is deprecated.  Use reactive_robot-msg:collision instead.")
  (collision m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <autodrive>) ostream)
  "Serializes a message object of type '<autodrive>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'collision) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <autodrive>) istream)
  "Deserializes a message object of type '<autodrive>"
    (cl:setf (cl:slot-value msg 'collision) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<autodrive>)))
  "Returns string type for a message object of type '<autodrive>"
  "reactive_robot/autodrive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'autodrive)))
  "Returns string type for a message object of type 'autodrive"
  "reactive_robot/autodrive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<autodrive>)))
  "Returns md5sum for a message object of type '<autodrive>"
  "ec9653804a13642f770edbe4a85843b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'autodrive)))
  "Returns md5sum for a message object of type 'autodrive"
  "ec9653804a13642f770edbe4a85843b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<autodrive>)))
  "Returns full string definition for message of type '<autodrive>"
  (cl:format cl:nil "bool collision~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'autodrive)))
  "Returns full string definition for message of type 'autodrive"
  (cl:format cl:nil "bool collision~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <autodrive>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <autodrive>))
  "Converts a ROS message object to a list"
  (cl:list 'autodrive
    (cl:cons ':collision (collision msg))
))
