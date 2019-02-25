;; Auto-generated. Do not edit!


(when (boundp 'reactive_robot::keyboard_override)
  (if (not (find-package "REACTIVE_ROBOT"))
    (make-package "REACTIVE_ROBOT"))
  (shadow 'keyboard_override (find-package "REACTIVE_ROBOT")))
(unless (find-package "REACTIVE_ROBOT::KEYBOARD_OVERRIDE")
  (make-package "REACTIVE_ROBOT::KEYBOARD_OVERRIDE"))
(unless (find-package "REACTIVE_ROBOT::KEYBOARD_OVERRIDEREQUEST")
  (make-package "REACTIVE_ROBOT::KEYBOARD_OVERRIDEREQUEST"))
(unless (find-package "REACTIVE_ROBOT::KEYBOARD_OVERRIDERESPONSE")
  (make-package "REACTIVE_ROBOT::KEYBOARD_OVERRIDERESPONSE"))

(in-package "ROS")





(defclass reactive_robot::keyboard_overrideRequest
  :super ros::object
  :slots ())

(defmethod reactive_robot::keyboard_overrideRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass reactive_robot::keyboard_overrideResponse
  :super ros::object
  :slots (_overridden ))

(defmethod reactive_robot::keyboard_overrideResponse
  (:init
   (&key
    ((:overridden __overridden) nil)
    )
   (send-super :init)
   (setq _overridden __overridden)
   self)
  (:overridden
   (&optional __overridden)
   (if __overridden (setq _overridden __overridden)) _overridden)
  (:serialization-length
   ()
   (+
    ;; bool _overridden
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _overridden
       (if _overridden (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _overridden
     (setq _overridden (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass reactive_robot::keyboard_override
  :super ros::object
  :slots ())

(setf (get reactive_robot::keyboard_override :md5sum-) "cb5aa041e0dc2943641ab2f5d3442948")
(setf (get reactive_robot::keyboard_override :datatype-) "reactive_robot/keyboard_override")
(setf (get reactive_robot::keyboard_override :request) reactive_robot::keyboard_overrideRequest)
(setf (get reactive_robot::keyboard_override :response) reactive_robot::keyboard_overrideResponse)

(defmethod reactive_robot::keyboard_overrideRequest
  (:response () (instance reactive_robot::keyboard_overrideResponse :init)))

(setf (get reactive_robot::keyboard_overrideRequest :md5sum-) "cb5aa041e0dc2943641ab2f5d3442948")
(setf (get reactive_robot::keyboard_overrideRequest :datatype-) "reactive_robot/keyboard_overrideRequest")
(setf (get reactive_robot::keyboard_overrideRequest :definition-)
      "---
bool overridden
")

(setf (get reactive_robot::keyboard_overrideResponse :md5sum-) "cb5aa041e0dc2943641ab2f5d3442948")
(setf (get reactive_robot::keyboard_overrideResponse :datatype-) "reactive_robot/keyboard_overrideResponse")
(setf (get reactive_robot::keyboard_overrideResponse :definition-)
      "---
bool overridden
")



(provide :reactive_robot/keyboard_override "cb5aa041e0dc2943641ab2f5d3442948")


