;; Auto-generated. Do not edit!


(when (boundp 'yeet_msgs::keyboard)
  (if (not (find-package "YEET_MSGS"))
    (make-package "YEET_MSGS"))
  (shadow 'keyboard (find-package "YEET_MSGS")))
(unless (find-package "YEET_MSGS::KEYBOARD")
  (make-package "YEET_MSGS::KEYBOARD"))

(in-package "ROS")
;;//! \htmlinclude keyboard.msg.html


(defclass yeet_msgs::keyboard
  :super ros::object
  :slots (_c ))

(defmethod yeet_msgs::keyboard
  (:init
   (&key
    ((:c __c) 0)
    )
   (send-super :init)
   (setq _c (round __c))
   self)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:serialization-length
   ()
   (+
    ;; char _c
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; char _c
       (write-byte _c s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; char _c
     (setq _c (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get yeet_msgs::keyboard :md5sum-) "503f37a585b485611c99195decce8bba")
(setf (get yeet_msgs::keyboard :datatype-) "yeet_msgs/keyboard")
(setf (get yeet_msgs::keyboard :definition-)
      "char c
")



(provide :yeet_msgs/keyboard "503f37a585b485611c99195decce8bba")


