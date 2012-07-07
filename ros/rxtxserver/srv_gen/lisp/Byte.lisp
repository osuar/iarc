; Auto-generated. Do not edit!


(cl:in-package rxtxserver-srv)


;//! \htmlinclude Byte-request.msg.html

(cl:defclass <Byte-request> (roslisp-msg-protocol:ros-message)
  ((outData
    :reader outData
    :initarg :outData
    :type cl:integer
    :initform 0))
)

(cl:defclass Byte-request (<Byte-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Byte-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Byte-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-srv:<Byte-request> is deprecated: use rxtxserver-srv:Byte-request instead.")))

(cl:ensure-generic-function 'outData-val :lambda-list '(m))
(cl:defmethod outData-val ((m <Byte-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-srv:outData-val is deprecated.  Use rxtxserver-srv:outData instead.")
  (outData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Byte-request>) ostream)
  "Serializes a message object of type '<Byte-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outData)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Byte-request>) istream)
  "Deserializes a message object of type '<Byte-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outData)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Byte-request>)))
  "Returns string type for a service object of type '<Byte-request>"
  "rxtxserver/ByteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Byte-request)))
  "Returns string type for a service object of type 'Byte-request"
  "rxtxserver/ByteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Byte-request>)))
  "Returns md5sum for a message object of type '<Byte-request>"
  "1363ee3da7a1536cdcdcdbac42798f92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Byte-request)))
  "Returns md5sum for a message object of type 'Byte-request"
  "1363ee3da7a1536cdcdcdbac42798f92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Byte-request>)))
  "Returns full string definition for message of type '<Byte-request>"
  (cl:format cl:nil "byte outData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Byte-request)))
  "Returns full string definition for message of type 'Byte-request"
  (cl:format cl:nil "byte outData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Byte-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Byte-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Byte-request
    (cl:cons ':outData (outData msg))
))
;//! \htmlinclude Byte-response.msg.html

(cl:defclass <Byte-response> (roslisp-msg-protocol:ros-message)
  ((outResponse
    :reader outResponse
    :initarg :outResponse
    :type cl:integer
    :initform 0))
)

(cl:defclass Byte-response (<Byte-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Byte-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Byte-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-srv:<Byte-response> is deprecated: use rxtxserver-srv:Byte-response instead.")))

(cl:ensure-generic-function 'outResponse-val :lambda-list '(m))
(cl:defmethod outResponse-val ((m <Byte-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-srv:outResponse-val is deprecated.  Use rxtxserver-srv:outResponse instead.")
  (outResponse m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Byte-response>) ostream)
  "Serializes a message object of type '<Byte-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outResponse)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Byte-response>) istream)
  "Deserializes a message object of type '<Byte-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outResponse)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Byte-response>)))
  "Returns string type for a service object of type '<Byte-response>"
  "rxtxserver/ByteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Byte-response)))
  "Returns string type for a service object of type 'Byte-response"
  "rxtxserver/ByteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Byte-response>)))
  "Returns md5sum for a message object of type '<Byte-response>"
  "1363ee3da7a1536cdcdcdbac42798f92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Byte-response)))
  "Returns md5sum for a message object of type 'Byte-response"
  "1363ee3da7a1536cdcdcdbac42798f92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Byte-response>)))
  "Returns full string definition for message of type '<Byte-response>"
  (cl:format cl:nil "byte outResponse~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Byte-response)))
  "Returns full string definition for message of type 'Byte-response"
  (cl:format cl:nil "byte outResponse~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Byte-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Byte-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Byte-response
    (cl:cons ':outResponse (outResponse msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Byte)))
  'Byte-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Byte)))
  'Byte-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Byte)))
  "Returns string type for a service object of type '<Byte>"
  "rxtxserver/Byte")