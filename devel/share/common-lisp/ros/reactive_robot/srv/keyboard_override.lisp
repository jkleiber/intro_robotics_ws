; Auto-generated. Do not edit!


(cl:in-package reactive_robot-srv)


;//! \htmlinclude keyboard_override-request.msg.html

(cl:defclass <keyboard_override-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass keyboard_override-request (<keyboard_override-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyboard_override-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyboard_override-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reactive_robot-srv:<keyboard_override-request> is deprecated: use reactive_robot-srv:keyboard_override-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyboard_override-request>) ostream)
  "Serializes a message object of type '<keyboard_override-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyboard_override-request>) istream)
  "Deserializes a message object of type '<keyboard_override-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyboard_override-request>)))
  "Returns string type for a service object of type '<keyboard_override-request>"
  "reactive_robot/keyboard_overrideRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard_override-request)))
  "Returns string type for a service object of type 'keyboard_override-request"
  "reactive_robot/keyboard_overrideRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyboard_override-request>)))
  "Returns md5sum for a message object of type '<keyboard_override-request>"
  "cb5aa041e0dc2943641ab2f5d3442948")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyboard_override-request)))
  "Returns md5sum for a message object of type 'keyboard_override-request"
  "cb5aa041e0dc2943641ab2f5d3442948")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyboard_override-request>)))
  "Returns full string definition for message of type '<keyboard_override-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyboard_override-request)))
  "Returns full string definition for message of type 'keyboard_override-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyboard_override-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyboard_override-request>))
  "Converts a ROS message object to a list"
  (cl:list 'keyboard_override-request
))
;//! \htmlinclude keyboard_override-response.msg.html

(cl:defclass <keyboard_override-response> (roslisp-msg-protocol:ros-message)
  ((overridden
    :reader overridden
    :initarg :overridden
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass keyboard_override-response (<keyboard_override-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyboard_override-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyboard_override-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reactive_robot-srv:<keyboard_override-response> is deprecated: use reactive_robot-srv:keyboard_override-response instead.")))

(cl:ensure-generic-function 'overridden-val :lambda-list '(m))
(cl:defmethod overridden-val ((m <keyboard_override-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reactive_robot-srv:overridden-val is deprecated.  Use reactive_robot-srv:overridden instead.")
  (overridden m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyboard_override-response>) ostream)
  "Serializes a message object of type '<keyboard_override-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'overridden) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyboard_override-response>) istream)
  "Deserializes a message object of type '<keyboard_override-response>"
    (cl:setf (cl:slot-value msg 'overridden) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyboard_override-response>)))
  "Returns string type for a service object of type '<keyboard_override-response>"
  "reactive_robot/keyboard_overrideResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard_override-response)))
  "Returns string type for a service object of type 'keyboard_override-response"
  "reactive_robot/keyboard_overrideResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyboard_override-response>)))
  "Returns md5sum for a message object of type '<keyboard_override-response>"
  "cb5aa041e0dc2943641ab2f5d3442948")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyboard_override-response)))
  "Returns md5sum for a message object of type 'keyboard_override-response"
  "cb5aa041e0dc2943641ab2f5d3442948")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyboard_override-response>)))
  "Returns full string definition for message of type '<keyboard_override-response>"
  (cl:format cl:nil "bool overridden~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyboard_override-response)))
  "Returns full string definition for message of type 'keyboard_override-response"
  (cl:format cl:nil "bool overridden~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyboard_override-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyboard_override-response>))
  "Converts a ROS message object to a list"
  (cl:list 'keyboard_override-response
    (cl:cons ':overridden (overridden msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'keyboard_override)))
  'keyboard_override-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'keyboard_override)))
  'keyboard_override-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard_override)))
  "Returns string type for a service object of type '<keyboard_override>"
  "reactive_robot/keyboard_override")