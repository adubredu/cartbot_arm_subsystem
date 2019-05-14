;; Auto-generated. Do not edit!


(when (boundp 'run_scripts::Command)
  (if (not (find-package "RUN_SCRIPTS"))
    (make-package "RUN_SCRIPTS"))
  (shadow 'Command (find-package "RUN_SCRIPTS")))
(unless (find-package "RUN_SCRIPTS::COMMAND")
  (make-package "RUN_SCRIPTS::COMMAND"))
(unless (find-package "RUN_SCRIPTS::COMMANDREQUEST")
  (make-package "RUN_SCRIPTS::COMMANDREQUEST"))
(unless (find-package "RUN_SCRIPTS::COMMANDRESPONSE")
  (make-package "RUN_SCRIPTS::COMMANDRESPONSE"))

(in-package "ROS")





(defclass run_scripts::CommandRequest
  :super ros::object
  :slots (_command ))

(defmethod run_scripts::CommandRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass run_scripts::CommandResponse
  :super ros::object
  :slots (_status ))

(defmethod run_scripts::CommandResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass run_scripts::Command
  :super ros::object
  :slots ())

(setf (get run_scripts::Command :md5sum-) "601bd226f166ed265256e4c57d16fd6c")
(setf (get run_scripts::Command :datatype-) "run_scripts/Command")
(setf (get run_scripts::Command :request) run_scripts::CommandRequest)
(setf (get run_scripts::Command :response) run_scripts::CommandResponse)

(defmethod run_scripts::CommandRequest
  (:response () (instance run_scripts::CommandResponse :init)))

(setf (get run_scripts::CommandRequest :md5sum-) "601bd226f166ed265256e4c57d16fd6c")
(setf (get run_scripts::CommandRequest :datatype-) "run_scripts/CommandRequest")
(setf (get run_scripts::CommandRequest :definition-)
      "string command
---
bool status
")

(setf (get run_scripts::CommandResponse :md5sum-) "601bd226f166ed265256e4c57d16fd6c")
(setf (get run_scripts::CommandResponse :datatype-) "run_scripts/CommandResponse")
(setf (get run_scripts::CommandResponse :definition-)
      "string command
---
bool status
")



(provide :run_scripts/Command "601bd226f166ed265256e4c57d16fd6c")


