; Auto-generated. Do not edit!


(cl:in-package reactive_robot-msg)


;//! \htmlinclude obstacle.msg.html

(cl:defclass <obstacle> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass obstacle (<obstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reactive_robot-msg:<obstacle> is deprecated: use reactive_robot-msg:obstacle instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reactive_robot-msg:state-val is deprecated.  Use reactive_robot-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<obstacle>)))
    "Constants for message type '<obstacle>"
  '((:EMPTY . 0)
    (:SYMMETRIC . 1)
    (:ASYMMETRIC . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'obstacle)))
    "Constants for message type 'obstacle"
  '((:EMPTY . 0)
    (:SYMMETRIC . 1)
    (:ASYMMETRIC . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle>) ostream)
  "Serializes a message object of type '<obstacle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle>) istream)
  "Deserializes a message object of type '<obstacle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle>)))
  "Returns string type for a message object of type '<obstacle>"
  "reactive_robot/obstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle)))
  "Returns string type for a message object of type 'obstacle"
  "reactive_robot/obstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle>)))
  "Returns md5sum for a message object of type '<obstacle>"
  "3fee00ef7d93fcd86cc6221b0265b0f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle)))
  "Returns md5sum for a message object of type 'obstacle"
  "3fee00ef7d93fcd86cc6221b0265b0f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle>)))
  "Returns full string definition for message of type '<obstacle>"
  (cl:format cl:nil "uint8 EMPTY = 0~%uint8 SYMMETRIC = 1~%uint8 ASYMMETRIC = 2~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle)))
  "Returns full string definition for message of type 'obstacle"
  (cl:format cl:nil "uint8 EMPTY = 0~%uint8 SYMMETRIC = 1~%uint8 ASYMMETRIC = 2~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle
    (cl:cons ':state (state msg))
))
