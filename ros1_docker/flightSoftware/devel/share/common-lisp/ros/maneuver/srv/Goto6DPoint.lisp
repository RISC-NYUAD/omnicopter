; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude Goto6DPoint-request.msg.html

(cl:defclass <Goto6DPoint-request> (roslisp-msg-protocol:ros-message)
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
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
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

(cl:defclass Goto6DPoint-request (<Goto6DPoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goto6DPoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goto6DPoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<Goto6DPoint-request> is deprecated: use maneuver-srv:Goto6DPoint-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:x-val is deprecated.  Use maneuver-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:y-val is deprecated.  Use maneuver-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:z-val is deprecated.  Use maneuver-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:roll-val is deprecated.  Use maneuver-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:pitch-val is deprecated.  Use maneuver-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:yaw-val is deprecated.  Use maneuver-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Goto6DPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration-val is deprecated.  Use maneuver-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goto6DPoint-request>) ostream)
  "Serializes a message object of type '<Goto6DPoint-request>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goto6DPoint-request>) istream)
  "Deserializes a message object of type '<Goto6DPoint-request>"
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
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goto6DPoint-request>)))
  "Returns string type for a service object of type '<Goto6DPoint-request>"
  "maneuver/Goto6DPointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goto6DPoint-request)))
  "Returns string type for a service object of type 'Goto6DPoint-request"
  "maneuver/Goto6DPointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goto6DPoint-request>)))
  "Returns md5sum for a message object of type '<Goto6DPoint-request>"
  "ceed9bb0a85fabd7e22e37d55fef957b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goto6DPoint-request)))
  "Returns md5sum for a message object of type 'Goto6DPoint-request"
  "ceed9bb0a85fabd7e22e37d55fef957b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goto6DPoint-request>)))
  "Returns full string definition for message of type '<Goto6DPoint-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 roll~%float32 pitch~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goto6DPoint-request)))
  "Returns full string definition for message of type 'Goto6DPoint-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 roll~%float32 pitch~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goto6DPoint-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goto6DPoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Goto6DPoint-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude Goto6DPoint-response.msg.html

(cl:defclass <Goto6DPoint-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Goto6DPoint-response (<Goto6DPoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goto6DPoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goto6DPoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<Goto6DPoint-response> is deprecated: use maneuver-srv:Goto6DPoint-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Goto6DPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goto6DPoint-response>) ostream)
  "Serializes a message object of type '<Goto6DPoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goto6DPoint-response>) istream)
  "Deserializes a message object of type '<Goto6DPoint-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goto6DPoint-response>)))
  "Returns string type for a service object of type '<Goto6DPoint-response>"
  "maneuver/Goto6DPointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goto6DPoint-response)))
  "Returns string type for a service object of type 'Goto6DPoint-response"
  "maneuver/Goto6DPointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goto6DPoint-response>)))
  "Returns md5sum for a message object of type '<Goto6DPoint-response>"
  "ceed9bb0a85fabd7e22e37d55fef957b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goto6DPoint-response)))
  "Returns md5sum for a message object of type 'Goto6DPoint-response"
  "ceed9bb0a85fabd7e22e37d55fef957b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goto6DPoint-response>)))
  "Returns full string definition for message of type '<Goto6DPoint-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goto6DPoint-response)))
  "Returns full string definition for message of type 'Goto6DPoint-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goto6DPoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goto6DPoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Goto6DPoint-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Goto6DPoint)))
  'Goto6DPoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Goto6DPoint)))
  'Goto6DPoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goto6DPoint)))
  "Returns string type for a service object of type '<Goto6DPoint>"
  "maneuver/Goto6DPoint")