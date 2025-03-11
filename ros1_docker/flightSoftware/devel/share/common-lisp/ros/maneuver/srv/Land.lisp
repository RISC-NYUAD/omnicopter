; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude Land-request.msg.html

(cl:defclass <Land-request> (roslisp-msg-protocol:ros-message)
  ((height_1
    :reader height_1
    :initarg :height_1
    :type cl:float
    :initform 0.0)
   (duration_1
    :reader duration_1
    :initarg :duration_1
    :type cl:float
    :initform 0.0)
   (height_2
    :reader height_2
    :initarg :height_2
    :type cl:float
    :initform 0.0)
   (duration_2
    :reader duration_2
    :initarg :duration_2
    :type cl:float
    :initform 0.0))
)

(cl:defclass Land-request (<Land-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Land-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Land-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<Land-request> is deprecated: use maneuver-srv:Land-request instead.")))

(cl:ensure-generic-function 'height_1-val :lambda-list '(m))
(cl:defmethod height_1-val ((m <Land-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:height_1-val is deprecated.  Use maneuver-srv:height_1 instead.")
  (height_1 m))

(cl:ensure-generic-function 'duration_1-val :lambda-list '(m))
(cl:defmethod duration_1-val ((m <Land-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration_1-val is deprecated.  Use maneuver-srv:duration_1 instead.")
  (duration_1 m))

(cl:ensure-generic-function 'height_2-val :lambda-list '(m))
(cl:defmethod height_2-val ((m <Land-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:height_2-val is deprecated.  Use maneuver-srv:height_2 instead.")
  (height_2 m))

(cl:ensure-generic-function 'duration_2-val :lambda-list '(m))
(cl:defmethod duration_2-val ((m <Land-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration_2-val is deprecated.  Use maneuver-srv:duration_2 instead.")
  (duration_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Land-request>) ostream)
  "Serializes a message object of type '<Land-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Land-request>) istream)
  "Deserializes a message object of type '<Land-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration_2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Land-request>)))
  "Returns string type for a service object of type '<Land-request>"
  "maneuver/LandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Land-request)))
  "Returns string type for a service object of type 'Land-request"
  "maneuver/LandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Land-request>)))
  "Returns md5sum for a message object of type '<Land-request>"
  "6caadcbcd5d54204c00f1a33593265c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Land-request)))
  "Returns md5sum for a message object of type 'Land-request"
  "6caadcbcd5d54204c00f1a33593265c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Land-request>)))
  "Returns full string definition for message of type '<Land-request>"
  (cl:format cl:nil "float32 height_1~%float32 duration_1~%float32 height_2~%float32 duration_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Land-request)))
  "Returns full string definition for message of type 'Land-request"
  (cl:format cl:nil "float32 height_1~%float32 duration_1~%float32 height_2~%float32 duration_2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Land-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Land-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Land-request
    (cl:cons ':height_1 (height_1 msg))
    (cl:cons ':duration_1 (duration_1 msg))
    (cl:cons ':height_2 (height_2 msg))
    (cl:cons ':duration_2 (duration_2 msg))
))
;//! \htmlinclude Land-response.msg.html

(cl:defclass <Land-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Land-response (<Land-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Land-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Land-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<Land-response> is deprecated: use maneuver-srv:Land-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Land-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Land-response>) ostream)
  "Serializes a message object of type '<Land-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Land-response>) istream)
  "Deserializes a message object of type '<Land-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Land-response>)))
  "Returns string type for a service object of type '<Land-response>"
  "maneuver/LandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Land-response)))
  "Returns string type for a service object of type 'Land-response"
  "maneuver/LandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Land-response>)))
  "Returns md5sum for a message object of type '<Land-response>"
  "6caadcbcd5d54204c00f1a33593265c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Land-response)))
  "Returns md5sum for a message object of type 'Land-response"
  "6caadcbcd5d54204c00f1a33593265c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Land-response>)))
  "Returns full string definition for message of type '<Land-response>"
  (cl:format cl:nil "bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Land-response)))
  "Returns full string definition for message of type 'Land-response"
  (cl:format cl:nil "bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Land-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Land-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Land-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Land)))
  'Land-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Land)))
  'Land-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Land)))
  "Returns string type for a service object of type '<Land>"
  "maneuver/Land")