
(cl:in-package :asdf)

(defsystem "walk_distance_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Walk" :depends-on ("_package_Walk"))
    (:file "_package_Walk" :depends-on ("_package"))
  ))