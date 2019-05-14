; Auto-generated. Do not edit!


(cl:in-package run_scripts-srv)


;//! \htmlinclude Command-request.msg.html

(cl:defclass <Command-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass Command-request (<Command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name run_scripts-srv:<Command-request> is deprecated: use run_scripts-srv:Command-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader run_scripts-srv:command-val is deprecated.  Use run_scripts-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command-request>) ostream)
  "Serializes a message object of type '<Command-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command-request>) istream)
  "Deserializes a message object of type '<Command-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command-request>)))
  "Returns string type for a service object of type '<Command-request>"
  "run_scripts/CommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command-request)))
  "Returns string type for a service object of type 'Command-request"
  "run_scripts/CommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command-request>)))
  "Returns md5sum for a message object of type '<Command-request>"
  "601bd226f166ed265256e4c57d16fd6c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command-request)))
  "Returns md5sum for a message object of type 'Command-request"
  "601bd226f166ed265256e4c57d16fd6c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command-request>)))
  "Returns full string definition for message of type '<Command-request>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command-request)))
  "Returns full string definition for message of type 'Command-request"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Command-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude Command-response.msg.html

(cl:defclass <Command-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Command-response (<Command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name run_scripts-srv:<Command-response> is deprecated: use run_scripts-srv:Command-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader run_scripts-srv:status-val is deprecated.  Use run_scripts-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command-response>) ostream)
  "Serializes a message object of type '<Command-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command-response>) istream)
  "Deserializes a message object of type '<Command-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command-response>)))
  "Returns string type for a service object of type '<Command-response>"
  "run_scripts/CommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command-response)))
  "Returns string type for a service object of type 'Command-response"
  "run_scripts/CommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command-response>)))
  "Returns md5sum for a message object of type '<Command-response>"
  "601bd226f166ed265256e4c57d16fd6c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command-response)))
  "Returns md5sum for a message object of type 'Command-response"
  "601bd226f166ed265256e4c57d16fd6c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command-response>)))
  "Returns full string definition for message of type '<Command-response>"
  (cl:format cl:nil "bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command-response)))
  "Returns full string definition for message of type 'Command-response"
  (cl:format cl:nil "bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Command-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Command)))
  'Command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Command)))
  'Command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command)))
  "Returns string type for a service object of type '<Command>"
  "run_scripts/Command")