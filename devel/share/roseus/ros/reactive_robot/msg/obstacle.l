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
  :slots (_state ))

(defmethod reactive_robot::obstacle
  (:init
   (&key
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get reactive_robot::obstacle :md5sum-) "3fee00ef7d93fcd86cc6221b0265b0f2")
(setf (get reactive_robot::obstacle :datatype-) "reactive_robot/obstacle")
(setf (get reactive_robot::obstacle :definition-)
      "uint8 EMPTY = 0
uint8 SYMMETRIC = 1
uint8 ASYMMETRIC = 2
uint8 state

")



(provide :reactive_robot/obstacle "3fee00ef7d93fcd86cc6221b0265b0f2")


