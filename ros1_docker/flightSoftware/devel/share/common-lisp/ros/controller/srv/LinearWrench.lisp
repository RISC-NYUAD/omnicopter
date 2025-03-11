; Auto-generated. Do not edit!


(cl:in-package controller-srv)


;//! \htmlinclude LinearWrench-request.msg.html

(cl:defclass <LinearWrench-request> (roslisp-msg-protocol:ros-message)
  ((fx1
    :reader fx1
    :initarg :fx1
    :type cl:float
    :initform 0.0)
   (fy1
    :reader fy1
    :initarg :fy1
    :type cl:float
    :initform 0.0)
   (fz1
    :reader fz1
    :initarg :fz1
    :type cl:float
    :initform 0.0)
   (fx2
    :reader fx2
    :initarg :fx2
    :type cl:float
    :initform 0.0)
   (fy2
    :reader fy2
    :initarg :fy2
    :type cl:float
    :initform 0.0)
   (fz2
    :reader fz2
    :initarg :fz2
    :type cl:float
    :initform 0.0)
   (ramp
    :reader ramp
    :initarg :ramp
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass LinearWrench-request (<LinearWrench-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinearWrench-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinearWrench-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<LinearWrench-request> is deprecated: use controller-srv:LinearWrench-request instead.")))

(cl:ensure-generic-function 'fx1-val :lambda-list '(m))
(cl:defmethod fx1-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fx1-val is deprecated.  Use controller-srv:fx1 instead.")
  (fx1 m))

(cl:ensure-generic-function 'fy1-val :lambda-list '(m))
(cl:defmethod fy1-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fy1-val is deprecated.  Use controller-srv:fy1 instead.")
  (fy1 m))

(cl:ensure-generic-function 'fz1-val :lambda-list '(m))
(cl:defmethod fz1-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fz1-val is deprecated.  Use controller-srv:fz1 instead.")
  (fz1 m))

(cl:ensure-generic-function 'fx2-val :lambda-list '(m))
(cl:defmethod fx2-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fx2-val is deprecated.  Use controller-srv:fx2 instead.")
  (fx2 m))

(cl:ensure-generic-function 'fy2-val :lambda-list '(m))
(cl:defmethod fy2-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fy2-val is deprecated.  Use controller-srv:fy2 instead.")
  (fy2 m))

(cl:ensure-generic-function 'fz2-val :lambda-list '(m))
(cl:defmethod fz2-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fz2-val is deprecated.  Use controller-srv:fz2 instead.")
  (fz2 m))

(cl:ensure-generic-function 'ramp-val :lambda-list '(m))
(cl:defmethod ramp-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:ramp-val is deprecated.  Use controller-srv:ramp instead.")
  (ramp m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <LinearWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:duration-val is deprecated.  Use controller-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinearWrench-request>) ostream)
  "Serializes a message object of type '<LinearWrench-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fx1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fy1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fz1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fx2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fy2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fz2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ramp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinearWrench-request>) istream)
  "Deserializes a message object of type '<LinearWrench-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fx1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fy1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fz1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fx2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fy2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fz2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ramp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinearWrench-request>)))
  "Returns string type for a service object of type '<LinearWrench-request>"
  "controller/LinearWrenchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearWrench-request)))
  "Returns string type for a service object of type 'LinearWrench-request"
  "controller/LinearWrenchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinearWrench-request>)))
  "Returns md5sum for a message object of type '<LinearWrench-request>"
  "676b2c4049e154bfe83fc0a28e11e13d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinearWrench-request)))
  "Returns md5sum for a message object of type 'LinearWrench-request"
  "676b2c4049e154bfe83fc0a28e11e13d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinearWrench-request>)))
  "Returns full string definition for message of type '<LinearWrench-request>"
  (cl:format cl:nil "float32 fx1~%float32 fy1~%float32 fz1~%float32 fx2~%float32 fy2~%float32 fz2~%float32 ramp~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinearWrench-request)))
  "Returns full string definition for message of type 'LinearWrench-request"
  (cl:format cl:nil "float32 fx1~%float32 fy1~%float32 fz1~%float32 fx2~%float32 fy2~%float32 fz2~%float32 ramp~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinearWrench-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinearWrench-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LinearWrench-request
    (cl:cons ':fx1 (fx1 msg))
    (cl:cons ':fy1 (fy1 msg))
    (cl:cons ':fz1 (fz1 msg))
    (cl:cons ':fx2 (fx2 msg))
    (cl:cons ':fy2 (fy2 msg))
    (cl:cons ':fz2 (fz2 msg))
    (cl:cons ':ramp (ramp msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude LinearWrench-response.msg.html

(cl:defclass <LinearWrench-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LinearWrench-response (<LinearWrench-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinearWrench-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinearWrench-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<LinearWrench-response> is deprecated: use controller-srv:LinearWrench-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <LinearWrench-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:status-val is deprecated.  Use controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinearWrench-response>) ostream)
  "Serializes a message object of type '<LinearWrench-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinearWrench-response>) istream)
  "Deserializes a message object of type '<LinearWrench-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinearWrench-response>)))
  "Returns string type for a service object of type '<LinearWrench-response>"
  "controller/LinearWrenchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearWrench-response)))
  "Returns string type for a service object of type 'LinearWrench-response"
  "controller/LinearWrenchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinearWrench-response>)))
  "Returns md5sum for a message object of type '<LinearWrench-response>"
  "676b2c4049e154bfe83fc0a28e11e13d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinearWrench-response)))
  "Returns md5sum for a message object of type 'LinearWrench-response"
  "676b2c4049e154bfe83fc0a28e11e13d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinearWrench-response>)))
  "Returns full string definition for message of type '<LinearWrench-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinearWrench-response)))
  "Returns full string definition for message of type 'LinearWrench-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinearWrench-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinearWrench-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LinearWrench-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LinearWrench)))
  'LinearWrench-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LinearWrench)))
  'LinearWrench-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearWrench)))
  "Returns string type for a service object of type '<LinearWrench>"
  "controller/LinearWrench")