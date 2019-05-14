
(cl:in-package :asdf)

(defsystem "run_scripts-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
  ))