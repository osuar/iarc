; Auto-generated. Do not edit!


(cl:in-package rxtxserver-msg)


;//! \htmlinclude distance.msg.html

(cl:defclass <distance> (roslisp-msg-protocol:ros-message)
  ((vertical
    :reader vertical
    :initarg :vertical
    :type cl:fixnum
    :initform 0))
)

(cl:defclass distance (<distance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <distance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'distance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-msg:<distance> is deprecated: use rxtxserver-msg:distance instead.")))

(cl:ensure-generic-function 'vertical-val :lambda-list '(m))
(cl:defmethod vertical-val ((m <distance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-msg:vertical-val is deprecated.  Use rxtxserver-msg:vertical instead.")
  (vertical m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <distance>) ostream)
  "Serializes a message object of type '<distance>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vertical)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vertical)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <distance>) istream)
  "Deserializes a message object of type '<distance>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vertical)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vertical)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<distance>)))
  "Returns string type for a message object of type '<distance>"
  "rxtxserver/distance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'distance)))
  "Returns string type for a message object of type 'distance"
  "rxtxserver/distance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<distance>)))
  "Returns md5sum for a message object of type '<distance>"
  "64bd46181ebfcf44c3f5a7c010852540")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'distance)))
  "Returns md5sum for a message object of type 'distance"
  "64bd46181ebfcf44c3f5a7c010852540")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<distance>)))
  "Returns full string definition for message of type '<distance>"
  (cl:format cl:nil "uint16 vertical~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'distance)))
  "Returns full string definition for message of type 'distance"
  (cl:format cl:nil "uint16 vertical~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <distance>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <distance>))
  "Converts a ROS message object to a list"
  (cl:list 'distance
    (cl:cons ':vertical (vertical msg))
))
