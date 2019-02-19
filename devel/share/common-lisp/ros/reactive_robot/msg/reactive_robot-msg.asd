
(cl:in-package :asdf)

(defsystem "reactive_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "autodrive" :depends-on ("_package_autodrive"))
    (:file "_package_autodrive" :depends-on ("_package"))
    (:file "collision" :depends-on ("_package_collision"))
    (:file "_package_collision" :depends-on ("_package"))
    (:file "obstacle" :depends-on ("_package_obstacle"))
    (:file "_package_obstacle" :depends-on ("_package"))
  ))