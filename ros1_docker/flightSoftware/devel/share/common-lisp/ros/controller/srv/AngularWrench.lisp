; Auto-generated. Do not edit!


(cl:in-package controller-srv)


;//! \htmlinclude AngularWrench-request.msg.html

(cl:defclass <AngularWrench-request> (roslisp-msg-protocol:ros-message)
  ((fz
    :reader fz
    :initarg :fz
    :type cl:float
    :initform 0.0)
   (phi1
    :reader phi1
    :initarg :phi1
    :type cl:float
    :initform 0.0)
   (phi2
    :reader phi2
    :initarg :phi2
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass AngularWrench-request (<AngularWrench-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AngularWrench-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AngularWrench-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<AngularWrench-request> is deprecated: use controller-srv:AngularWrench-request instead.")))

(cl:ensure-generic-function 'fz-val :lambda-list '(m))
(cl:defmethod fz-val ((m <AngularWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:fz-val is deprecated.  Use controller-srv:fz instead.")
  (fz m))

(cl:ensure-generic-function 'phi1-val :lambda-list '(m))
(cl:defmethod phi1-val ((m <AngularWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:phi1-val is deprecated.  Use controller-srv:phi1 instead.")
  (phi1 m))

(cl:ensure-generic-function 'phi2-val :lambda-list '(m))
(cl:defmethod phi2-val ((m <AngularWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:phi2-val is deprecated.  Use controller-srv:phi2 instead.")
  (phi2 m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <AngularWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:duration-val is deprecated.  Use controller-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AngularWrench-request>) ostream)
  "Serializes a message object of type '<AngularWrench-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'phi1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'phi2))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AngularWrench-request>) istream)
  "Deserializes a message object of type '<AngularWrench-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AngularWrench-request>)))
  "Returns string type for a service object of type '<AngularWrench-request>"
  "controller/AngularWrenchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngularWrench-request)))
  "Returns string type for a service object of type 'AngularWrench-request"
  "controller/AngularWrenchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AngularWrench-request>)))
  "Returns md5sum for a message object of type '<AngularWrench-request>"
  "bfbfde29636da2ef0700941fae523d7c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AngularWrench-request)))
  "Returns md5sum for a message object of type 'AngularWrench-request"
  "bfbfde29636da2ef0700941fae523d7c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AngularWrench-request>)))
  "Returns full string definition for message of type '<AngularWrench-request>"
  (cl:format cl:nil "float32 fz~%float32 phi1~%float32 phi2~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AngularWrench-request)))
  "Returns full string definition for message of type 'AngularWrench-request"
  (cl:format cl:nil "float32 fz~%float32 phi1~%float32 phi2~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AngularWrench-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AngularWrench-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AngularWrench-request
    (cl:cons ':fz (fz msg))
    (cl:cons ':phi1 (phi1 msg))
    (cl:cons ':phi2 (phi2 msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude AngularWrench-response.msg.html

(cl:defclass <AngularWrench-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AngularWrench-response (<AngularWrench-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AngularWrench-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AngularWrench-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<AngularWrench-response> is deprecated: use controller-srv:AngularWrench-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <AngularWrench-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:status-val is deprecated.  Use controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AngularWrench-response>) ostream)
  "Serializes a message object of type '<AngularWrench-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AngularWrench-response>) istream)
  "Deserializes a message object of type '<AngularWrench-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AngularWrench-response>)))
  "Returns string type for a service object of type '<AngularWrench-response>"
  "controller/AngularWrenchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngularWrench-response)))
  "Returns string type for a service object of type 'AngularWrench-response"
  "controller/AngularWrenchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AngularWrench-response>)))
  "Returns md5sum for a message object of type '<AngularWrench-response>"
  "bfbfde29636da2ef0700941fae523d7c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AngularWrench-response)))
  "Returns md5sum for a message object of type 'AngularWrench-response"
  "bfbfde29636da2ef0700941fae523d7c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AngularWrench-response>)))
  "Returns full string definition for message of type '<AngularWrench-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AngularWrench-response)))
  "Returns full string definition for message of type 'AngularWrench-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AngularWrench-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AngularWrench-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AngularWrench-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AngularWrench)))
  'AngularWrench-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AngularWrench)))
  'AngularWrench-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngularWrench)))
  "Returns string type for a service object of type '<AngularWrench>"
  "controller/AngularWrench")