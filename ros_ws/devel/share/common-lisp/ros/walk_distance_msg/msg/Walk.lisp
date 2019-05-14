; Auto-generated. Do not edit!


(cl:in-package walk_distance_msg-msg)


;//! \htmlinclude Walk.msg.html

(cl:defclass <Walk> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass Walk (<Walk>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Walk>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Walk)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walk_distance_msg-msg:<Walk> is deprecated: use walk_distance_msg-msg:Walk instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walk_distance_msg-msg:direction-val is deprecated.  Use walk_distance_msg-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <Walk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walk_distance_msg-msg:distance-val is deprecated.  Use walk_distance_msg-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Walk>) ostream)
  "Serializes a message object of type '<Walk>"
  (cl:let* ((signed (cl:slot-value msg 'direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Walk>) istream)
  "Deserializes a message object of type '<Walk>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Walk>)))
  "Returns string type for a message object of type '<Walk>"
  "walk_distance_msg/Walk")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Walk)))
  "Returns string type for a message object of type 'Walk"
  "walk_distance_msg/Walk")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Walk>)))
  "Returns md5sum for a message object of type '<Walk>"
  "fa939a7f5aefe801fd8d555219b82603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Walk)))
  "Returns md5sum for a message object of type 'Walk"
  "fa939a7f5aefe801fd8d555219b82603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Walk>)))
  "Returns full string definition for message of type '<Walk>"
  (cl:format cl:nil "int32 direction~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Walk)))
  "Returns full string definition for message of type 'Walk"
  (cl:format cl:nil "int32 direction~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Walk>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Walk>))
  "Converts a ROS message object to a list"
  (cl:list 'Walk
    (cl:cons ':direction (direction msg))
    (cl:cons ':distance (distance msg))
))
