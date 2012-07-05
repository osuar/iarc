; Auto-generated. Do not edit!


(cl:in-package rxtxserver-msg)


;//! \htmlinclude status.msg.html

(cl:defclass <status> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass status (<status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-msg:<status> is deprecated: use rxtxserver-msg:status instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-msg:status-val is deprecated.  Use rxtxserver-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <status>) ostream)
  "Serializes a message object of type '<status>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <status>) istream)
  "Deserializes a message object of type '<status>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<status>)))
  "Returns string type for a message object of type '<status>"
  "rxtxserver/status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'status)))
  "Returns string type for a message object of type 'status"
  "rxtxserver/status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<status>)))
  "Returns md5sum for a message object of type '<status>"
  "8695a687dd99fd6c4e83bb483ce1c397")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'status)))
  "Returns md5sum for a message object of type 'status"
  "8695a687dd99fd6c4e83bb483ce1c397")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<status>)))
  "Returns full string definition for message of type '<status>"
  (cl:format cl:nil "char status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'status)))
  "Returns full string definition for message of type 'status"
  (cl:format cl:nil "char status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <status>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <status>))
  "Converts a ROS message object to a list"
  (cl:list 'status
    (cl:cons ':status (status msg))
))
