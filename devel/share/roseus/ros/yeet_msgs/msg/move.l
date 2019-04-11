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
  :slots (_todo ))

(defmethod yeet_msgs::move
  (:init
   (&key
    ((:todo __todo) 0)
    )
   (send-super :init)
   (setq _todo (round __todo))
   self)
  (:todo
   (&optional __todo)
   (if __todo (setq _todo __todo)) _todo)
  (:serialization-length
   ()
   (+
    ;; int8 _todo
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _todo
       (write-byte _todo s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _todo
     (setq _todo (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _todo 127) (setq _todo (- _todo 256)))
   ;;
   self)
  )

(setf (get yeet_msgs::move :md5sum-) "cf55fd99d4d1be34562aaf532f8ee9a7")
(setf (get yeet_msgs::move :datatype-) "yeet_msgs/move")
(setf (get yeet_msgs::move :definition-)
      "int8 todo
#test
")



(provide :yeet_msgs/move "cf55fd99d4d1be34562aaf532f8ee9a7")


