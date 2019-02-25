;; Auto-generated. Do not edit!


(when (boundp 'reactive_robot::obstacle)
  (if (not (find-package "REACTIVE_ROBOT"))
    (make-package "REACTIVE_ROBOT"))
  (shadow 'obstacle (find-package "REACTIVE_ROBOT")))
(unless (find-package "REACTIVE_ROBOT::OBSTACLE")
  (make-package "REACTIVE_ROBOT::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude obstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*EMPTY*" (find-package "REACTIVE_ROBOT::OBSTACLE"))
(shadow '*EMPTY* (find-package "REACTIVE_ROBOT::OBSTACLE"))
(defconstant reactive_robot::obstacle::*EMPTY* 0)
(intern "*SYMMETRIC*" (find-package "REACTIVE_ROBOT::OBSTACLE"))
(shadow '*SYMMETRIC* (find-package "REACTIVE_ROBOT::OBSTACLE"))
(defconstant reactive_robot::obstacle::*SYMMETRIC* 1)
(intern "*ASYMMETRIC*" (find-package "REACTIVE_ROBOT::OBSTACLE"))
(shadow '*ASYMMETRIC* (find-package "REACTIVE_ROBOT::OBSTACLE"))
(defconstant reactive_robot::obstacle::*ASYMMETRIC* 2)
(defclass reactive_robot::obstacle
  :super ros::object
  :slots (_state _angle _distance _drive ))

(defmethod reactive_robot::obstacle
  (:init
   (&key
    ((:state __state) 0)
    ((:angle __angle) 0.0)
    ((:distance __distance) 0.0)
    ((:drive __drive) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _state (round __state))
   (setq _angle (float __angle))
   (setq _distance (float __distance))
   (setq _drive __drive)
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:drive
   (&rest __drive)
   (if (keywordp (car __drive))
       (send* _drive __drive)
     (progn
       (if __drive (setq _drive (car __drive)))
       _drive)))
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ;; float32 _angle
    4
    ;; float32 _distance
    4
    ;; geometry_msgs/Twist _drive
    (send _drive :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Twist _drive
       (send _drive :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Twist _drive
     (send _drive :deserialize buf ptr-) (incf ptr- (send _drive :serialization-length))
   ;;
   self)
  )

(setf (get reactive_robot::obstacle :md5sum-) "9def4b737c98a8f46cacc0980db0cf56")
(setf (get reactive_robot::obstacle :datatype-) "reactive_robot/obstacle")
(setf (get reactive_robot::obstacle :definition-)
      "uint8 EMPTY = 0
uint8 SYMMETRIC = 1
uint8 ASYMMETRIC = 2
uint8 state
float32 angle
float32 distance
geometry_msgs/Twist drive
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :reactive_robot/obstacle "9def4b737c98a8f46cacc0980db0cf56")


