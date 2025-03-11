; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude GotoPoint-request.msg.html

(cl:defclass <GotoPoint-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass GotoPoint-request (<GotoPoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GotoPoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GotoPoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<GotoPoint-request> is deprecated: use maneuver-srv:GotoPoint-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GotoPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:x-val is deprecated.  Use maneuver-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GotoPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:y-val is deprecated.  Use maneuver-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <GotoPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:z-val is deprecated.  Use maneuver-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GotoPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:yaw-val is deprecated.  Use maneuver-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <GotoPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration-val is deprecated.  Use maneuver-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GotoPoint-request>) ostream)
  "Serializes a message object of type '<GotoPoint-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GotoPoint-request>) istream)
  "Deserializes a message object of type '<GotoPoint-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GotoPoint-request>)))
  "Returns string type for a service object of type '<GotoPoint-request>"
  "maneuver/GotoPointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GotoPoint-request)))
  "Returns string type for a service object of type 'GotoPoint-request"
  "maneuver/GotoPointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GotoPoint-request>)))
  "Returns md5sum for a message object of type '<GotoPoint-request>"
  "13a22628e95c1b6dce035f4797b346a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GotoPoint-request)))
  "Returns md5sum for a message object of type 'GotoPoint-request"
  "13a22628e95c1b6dce035f4797b346a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GotoPoint-request>)))
  "Returns full string definition for message of type '<GotoPoint-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GotoPoint-request)))
  "Returns full string definition for message of type 'GotoPoint-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GotoPoint-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GotoPoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GotoPoint-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude GotoPoint-response.msg.html

(cl:defclass <GotoPoint-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GotoPoint-response (<GotoPoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GotoPoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GotoPoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<GotoPoint-response> is deprecated: use maneuver-srv:GotoPoint-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GotoPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GotoPoint-response>) ostream)
  "Serializes a message object of type '<GotoPoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GotoPoint-response>) istream)
  "Deserializes a message object of type '<GotoPoint-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GotoPoint-response>)))
  "Returns string type for a service object of type '<GotoPoint-response>"
  "maneuver/GotoPointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GotoPoint-response)))
  "Returns string type for a service object of type 'GotoPoint-response"
  "maneuver/GotoPointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GotoPoint-response>)))
  "Returns md5sum for a message object of type '<GotoPoint-response>"
  "13a22628e95c1b6dce035f4797b346a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GotoPoint-response)))
  "Returns md5sum for a message object of type 'GotoPoint-response"
  "13a22628e95c1b6dce035f4797b346a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GotoPoint-response>)))
  "Returns full string definition for message of type '<GotoPoint-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GotoPoint-response)))
  "Returns full string definition for message of type 'GotoPoint-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GotoPoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GotoPoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GotoPoint-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GotoPoint)))
  'GotoPoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GotoPoint)))
  'GotoPoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GotoPoint)))
  "Returns string type for a service object of type '<GotoPoint>"
  "maneuver/GotoPoint")