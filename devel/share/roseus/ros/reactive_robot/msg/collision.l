;; Auto-generated. Do not edit!


(when (boundp 'reactive_robot::collision)
  (if (not (find-package "REACTIVE_ROBOT"))
    (make-package "REACTIVE_ROBOT"))
  (shadow 'collision (find-package "REACTIVE_ROBOT")))
(unless (find-package "REACTIVE_ROBOT::COLLISION")
  (make-package "REACTIVE_ROBOT::COLLISION"))

(in-package "ROS")
;;//! \htmlinclude collision.msg.html


(defclass reactive_robot::collision
  :super ros::object
  :slots (_collision ))

(defmethod reactive_robot::collision
  (:init
   (&key
    ((:collision __collision) nil)
    )
   (send-super :init)
   (setq _collision __collision)
   self)
  (:collision
   (&optional __collision)
   (if __collision (setq _collision __collision)) _collision)
  (:serialization-length
   ()
   (+
    ;; bool _collision
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _collision
       (if _collision (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _collision
     (setq _collision (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get reactive_robot::collision :md5sum-) "ec9653804a13642f770edbe4a85843b2")
(setf (get reactive_robot::collision :datatype-) "reactive_robot/collision")
(setf (get reactive_robot::collision :definition-)
      "bool collision

")



(provide :reactive_robot/collision "ec9653804a13642f770edbe4a85843b2")


