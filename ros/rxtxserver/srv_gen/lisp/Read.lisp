; Auto-generated. Do not edit!


(cl:in-package rxtxserver-srv)


;//! \htmlinclude Read-request.msg.html

(cl:defclass <Read-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Read-request (<Read-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Read-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Read-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-srv:<Read-request> is deprecated: use rxtxserver-srv:Read-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Read-request>) ostream)
  "Serializes a message object of type '<Read-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Read-request>) istream)
  "Deserializes a message object of type '<Read-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Read-request>)))
  "Returns string type for a service object of type '<Read-request>"
  "rxtxserver/ReadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Read-request)))
  "Returns string type for a service object of type 'Read-request"
  "rxtxserver/ReadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Read-request>)))
  "Returns md5sum for a message object of type '<Read-request>"
  "10545a036ebd67aa10ae2e15db0715dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Read-request)))
  "Returns md5sum for a message object of type 'Read-request"
  "10545a036ebd67aa10ae2e15db0715dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Read-request>)))
  "Returns full string definition for message of type '<Read-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Read-request)))
  "Returns full string definition for message of type 'Read-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Read-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Read-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Read-request
))
;//! \htmlinclude Read-response.msg.html

(cl:defclass <Read-response> (roslisp-msg-protocol:ros-message)
  ((outData
    :reader outData
    :initarg :outData
    :type cl:integer
    :initform 0))
)

(cl:defclass Read-response (<Read-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Read-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Read-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-srv:<Read-response> is deprecated: use rxtxserver-srv:Read-response instead.")))

(cl:ensure-generic-function 'outData-val :lambda-list '(m))
(cl:defmethod outData-val ((m <Read-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-srv:outData-val is deprecated.  Use rxtxserver-srv:outData instead.")
  (outData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Read-response>) ostream)
  "Serializes a message object of type '<Read-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outData)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Read-response>) istream)
  "Deserializes a message object of type '<Read-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outData)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Read-response>)))
  "Returns string type for a service object of type '<Read-response>"
  "rxtxserver/ReadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Read-response)))
  "Returns string type for a service object of type 'Read-response"
  "rxtxserver/ReadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Read-response>)))
  "Returns md5sum for a message object of type '<Read-response>"
  "10545a036ebd67aa10ae2e15db0715dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Read-response)))
  "Returns md5sum for a message object of type 'Read-response"
  "10545a036ebd67aa10ae2e15db0715dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Read-response>)))
  "Returns full string definition for message of type '<Read-response>"
  (cl:format cl:nil "byte outData~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Read-response)))
  "Returns full string definition for message of type 'Read-response"
  (cl:format cl:nil "byte outData~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Read-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Read-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Read-response
    (cl:cons ':outData (outData msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Read)))
  'Read-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Read)))
  'Read-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Read)))
  "Returns string type for a service object of type '<Read>"
  "rxtxserver/Read")