
(cl:in-package :asdf)

(defsystem "reactive_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "collision" :depends-on ("_package_collision"))
    (:file "_package_collision" :depends-on ("_package"))
  ))