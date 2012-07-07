; Auto-generated. Do not edit!


(cl:in-package rxtxserver-msg)


;//! \htmlinclude AngularPos.msg.html

(cl:defclass <AngularPos> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (z
    :reader z
    :initarg :z
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AngularPos (<AngularPos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AngularPos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AngularPos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rxtxserver-msg:<AngularPos> is deprecated: use rxtxserver-msg:AngularPos instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <AngularPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-msg:x-val is deprecated.  Use rxtxserver-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <AngularPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-msg:y-val is deprecated.  Use rxtxserver-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <AngularPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rxtxserver-msg:z-val is deprecated.  Use rxtxserver-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AngularPos>) ostream)
  "Serializes a message object of type '<AngularPos>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AngularPos>) istream)
  "Deserializes a message object of type '<AngularPos>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AngularPos>)))
  "Returns string type for a message object of type '<AngularPos>"
  "rxtxserver/AngularPos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngularPos)))
  "Returns string type for a message object of type 'AngularPos"
  "rxtxserver/AngularPos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AngularPos>)))
  "Returns md5sum for a message object of type '<AngularPos>"
  "85729383565f7e059d4a213b3db1317b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AngularPos)))
  "Returns md5sum for a message object of type 'AngularPos"
  "85729383565f7e059d4a213b3db1317b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AngularPos>)))
  "Returns full string definition for message of type '<AngularPos>"
  (cl:format cl:nil "int16 x~%int16 y~%int16 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AngularPos)))
  "Returns full string definition for message of type 'AngularPos"
  (cl:format cl:nil "int16 x~%int16 y~%int16 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AngularPos>))
  (cl:+ 0
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AngularPos>))
  "Converts a ROS message object to a list"
  (cl:list 'AngularPos
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
