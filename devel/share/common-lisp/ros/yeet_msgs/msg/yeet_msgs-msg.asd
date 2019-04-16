
(cl:in-package :asdf)

(defsystem "yeet_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "keyboard" :depends-on ("_package_keyboard"))
    (:file "_package_keyboard" :depends-on ("_package"))
    (:file "move" :depends-on ("_package_move"))
    (:file "_package_move" :depends-on ("_package"))
    (:file "node" :depends-on ("_package_node"))
    (:file "_package_node" :depends-on ("_package"))
  ))