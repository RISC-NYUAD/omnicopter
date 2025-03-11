; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude LiftOff-request.msg.html

(cl:defclass <LiftOff-request> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass LiftOff-request (<LiftOff-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LiftOff-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LiftOff-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<LiftOff-request> is deprecated: use maneuver-srv:LiftOff-request instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <LiftOff-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:height-val is deprecated.  Use maneuver-srv:height instead.")
  (height m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <LiftOff-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration-val is deprecated.  Use maneuver-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LiftOff-request>) ostream)
  "Serializes a message object of type '<LiftOff-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LiftOff-request>) istream)
  "Deserializes a message object of type '<LiftOff-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LiftOff-request>)))
  "Returns string type for a service object of type '<LiftOff-request>"
  "maneuver/LiftOffRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiftOff-request)))
  "Returns string type for a service object of type 'LiftOff-request"
  "maneuver/LiftOffRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LiftOff-request>)))
  "Returns md5sum for a message object of type '<LiftOff-request>"
  "9e16cfae3e3728d84a285e6768841653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LiftOff-request)))
  "Returns md5sum for a message object of type 'LiftOff-request"
  "9e16cfae3e3728d84a285e6768841653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LiftOff-request>)))
  "Returns full string definition for message of type '<LiftOff-request>"
  (cl:format cl:nil "float32 height~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LiftOff-request)))
  "Returns full string definition for message of type 'LiftOff-request"
  (cl:format cl:nil "float32 height~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LiftOff-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LiftOff-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LiftOff-request
    (cl:cons ':height (height msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude LiftOff-response.msg.html

(cl:defclass <LiftOff-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LiftOff-response (<LiftOff-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LiftOff-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LiftOff-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<LiftOff-response> is deprecated: use maneuver-srv:LiftOff-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <LiftOff-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LiftOff-response>) ostream)
  "Serializes a message object of type '<LiftOff-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LiftOff-response>) istream)
  "Deserializes a message object of type '<LiftOff-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LiftOff-response>)))
  "Returns string type for a service object of type '<LiftOff-response>"
  "maneuver/LiftOffResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiftOff-response)))
  "Returns string type for a service object of type 'LiftOff-response"
  "maneuver/LiftOffResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LiftOff-response>)))
  "Returns md5sum for a message object of type '<LiftOff-response>"
  "9e16cfae3e3728d84a285e6768841653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LiftOff-response)))
  "Returns md5sum for a message object of type 'LiftOff-response"
  "9e16cfae3e3728d84a285e6768841653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LiftOff-response>)))
  "Returns full string definition for message of type '<LiftOff-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LiftOff-response)))
  "Returns full string definition for message of type 'LiftOff-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LiftOff-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LiftOff-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LiftOff-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LiftOff)))
  'LiftOff-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LiftOff)))
  'LiftOff-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiftOff)))
  "Returns string type for a service object of type '<LiftOff>"
  "maneuver/LiftOff")