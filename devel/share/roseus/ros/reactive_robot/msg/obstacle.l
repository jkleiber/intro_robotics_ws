;; Auto-generated. Do not edit!


(when (boundp 'reactive_robot::obstacle)
  (if (not (find-package "REACTIVE_ROBOT"))
    (make-package "REACTIVE_ROBOT"))
  (shadow 'obstacle (find-package "REACTIVE_ROBOT")))
(unless (find-package "REACTIVE_ROBOT::OBSTACLE")
  (make-package "REACTIVE_ROBOT::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude obstacle.msg.html


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
  :slots (_state _distance ))

(defmethod reactive_robot::obstacle
  (:init
   (&key
    ((:state __state) 0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _state (round __state))
   (setq _distance (float __distance))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get reactive_robot::obstacle :md5sum-) "6b7c0fc076289782d62bfcafc053166e")
(setf (get reactive_robot::obstacle :datatype-) "reactive_robot/obstacle")
(setf (get reactive_robot::obstacle :definition-)
      "uint8 EMPTY = 0
uint8 SYMMETRIC = 1
uint8 ASYMMETRIC = 2
uint8 state
float64 distance
")



(provide :reactive_robot/obstacle "6b7c0fc076289782d62bfcafc053166e")


