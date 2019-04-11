;; Auto-generated. Do not edit!


(when (boundp 'yeet_msgs::move)
  (if (not (find-package "YEET_MSGS"))
    (make-package "YEET_MSGS"))
  (shadow 'move (find-package "YEET_MSGS")))
(unless (find-package "YEET_MSGS::MOVE")
  (make-package "YEET_MSGS::MOVE"))

(in-package "ROS")
;;//! \htmlinclude move.msg.html


(defclass yeet_msgs::move
  :super ros::object
  :slots (_drive _turn ))

(defmethod yeet_msgs::move
  (:init
   (&key
    ((:drive __drive) 0)
    ((:turn __turn) 0)
    )
   (send-super :init)
   (setq _drive (round __drive))
   (setq _turn (round __turn))
   self)
  (:drive
   (&optional __drive)
   (if __drive (setq _drive __drive)) _drive)
  (:turn
   (&optional __turn)
   (if __turn (setq _turn __turn)) _turn)
  (:serialization-length
   ()
   (+
    ;; int8 _drive
    1
    ;; int8 _turn
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _drive
       (write-byte _drive s)
     ;; int8 _turn
       (write-byte _turn s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _drive
     (setq _drive (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _drive 127) (setq _drive (- _drive 256)))
   ;; int8 _turn
     (setq _turn (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _turn 127) (setq _turn (- _turn 256)))
   ;;
   self)
  )

(setf (get yeet_msgs::move :md5sum-) "8f1cddb8ef3caea21484489b8f1096b9")
(setf (get yeet_msgs::move :datatype-) "yeet_msgs/move")
(setf (get yeet_msgs::move :definition-)
      "int8 drive
int8 turn

")



(provide :yeet_msgs/move "8f1cddb8ef3caea21484489b8f1096b9")


