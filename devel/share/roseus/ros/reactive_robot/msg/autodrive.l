;; Auto-generated. Do not edit!


(when (boundp 'reactive_robot::autodrive)
  (if (not (find-package "REACTIVE_ROBOT"))
    (make-package "REACTIVE_ROBOT"))
  (shadow 'autodrive (find-package "REACTIVE_ROBOT")))
(unless (find-package "REACTIVE_ROBOT::AUTODRIVE")
  (make-package "REACTIVE_ROBOT::AUTODRIVE"))

(in-package "ROS")
;;//! \htmlinclude autodrive.msg.html


(defclass reactive_robot::autodrive
  :super ros::object
  :slots (_drive _turn_angle ))

(defmethod reactive_robot::autodrive
  (:init
   (&key
    ((:drive __drive) nil)
    ((:turn_angle __turn_angle) 0)
    )
   (send-super :init)
   (setq _drive __drive)
   (setq _turn_angle (round __turn_angle))
   self)
  (:drive
   (&optional __drive)
   (if __drive (setq _drive __drive)) _drive)
  (:turn_angle
   (&optional __turn_angle)
   (if __turn_angle (setq _turn_angle __turn_angle)) _turn_angle)
  (:serialization-length
   ()
   (+
    ;; bool _drive
    1
    ;; int8 _turn_angle
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _drive
       (if _drive (write-byte -1 s) (write-byte 0 s))
     ;; int8 _turn_angle
       (write-byte _turn_angle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _drive
     (setq _drive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _turn_angle
     (setq _turn_angle (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _turn_angle 127) (setq _turn_angle (- _turn_angle 256)))
   ;;
   self)
  )

(setf (get reactive_robot::autodrive :md5sum-) "ad06341ef70ac04503af948ffea2ec63")
(setf (get reactive_robot::autodrive :datatype-) "reactive_robot/autodrive")
(setf (get reactive_robot::autodrive :definition-)
      "bool drive
int8 turn_angle

")



(provide :reactive_robot/autodrive "ad06341ef70ac04503af948ffea2ec63")


