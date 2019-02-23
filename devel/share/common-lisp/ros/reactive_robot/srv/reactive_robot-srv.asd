
(cl:in-package :asdf)

(defsystem "reactive_robot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "keyboard_override" :depends-on ("_package_keyboard_override"))
    (:file "_package_keyboard_override" :depends-on ("_package"))
  ))