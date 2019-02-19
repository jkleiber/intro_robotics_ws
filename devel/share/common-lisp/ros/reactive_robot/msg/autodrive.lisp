; Auto-generated. Do not edit!


(cl:in-package reactive_robot-msg)


;//! \htmlinclude autodrive.msg.html

(cl:defclass <autodrive> (roslisp-msg-protocol:ros-message)
  ((drive
    :reader drive
    :initarg :drive
    :type cl:boolean
    :initform cl:nil)
   (turn_angle
    :reader turn_angle
    :initarg :turn_angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass autodrive (<autodrive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <autodrive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'autodrive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reactive_robot-msg:<autodrive> is deprecated: use reactive_robot-msg:autodrive instead.")))

(cl:ensure-generic-function 'drive-val :lambda-list '(m))
(cl:defmethod drive-val ((m <autodrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reactive_robot-msg:drive-val is deprecated.  Use reactive_robot-msg:drive instead.")
  (drive m))

(cl:ensure-generic-function 'turn_angle-val :lambda-list '(m))
(cl:defmethod turn_angle-val ((m <autodrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reactive_robot-msg:turn_angle-val is deprecated.  Use reactive_robot-msg:turn_angle instead.")
  (turn_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <autodrive>) ostream)
  "Serializes a message object of type '<autodrive>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'drive) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'turn_angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <autodrive>) istream)
  "Deserializes a message object of type '<autodrive>"
    (cl:setf (cl:slot-value msg 'drive) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'turn_angle) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
  "ad06341ef70ac04503af948ffea2ec63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'autodrive)))
  "Returns md5sum for a message object of type 'autodrive"
  "ad06341ef70ac04503af948ffea2ec63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<autodrive>)))
  "Returns full string definition for message of type '<autodrive>"
  (cl:format cl:nil "bool drive~%int8 turn_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'autodrive)))
  "Returns full string definition for message of type 'autodrive"
  (cl:format cl:nil "bool drive~%int8 turn_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <autodrive>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <autodrive>))
  "Converts a ROS message object to a list"
  (cl:list 'autodrive
    (cl:cons ':drive (drive msg))
    (cl:cons ':turn_angle (turn_angle msg))
))
