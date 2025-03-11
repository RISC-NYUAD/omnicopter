; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude RotateTo-request.msg.html

(cl:defclass <RotateTo-request> (roslisp-msg-protocol:ros-message)
  ((roll
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

(cl:defclass RotateTo-request (<RotateTo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotateTo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotateTo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<RotateTo-request> is deprecated: use maneuver-srv:RotateTo-request instead.")))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <RotateTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:roll-val is deprecated.  Use maneuver-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <RotateTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:pitch-val is deprecated.  Use maneuver-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <RotateTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:yaw-val is deprecated.  Use maneuver-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <RotateTo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration-val is deprecated.  Use maneuver-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotateTo-request>) ostream)
  "Serializes a message object of type '<RotateTo-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotateTo-request>) istream)
  "Deserializes a message object of type '<RotateTo-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotateTo-request>)))
  "Returns string type for a service object of type '<RotateTo-request>"
  "maneuver/RotateToRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateTo-request)))
  "Returns string type for a service object of type 'RotateTo-request"
  "maneuver/RotateToRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotateTo-request>)))
  "Returns md5sum for a message object of type '<RotateTo-request>"
  "e91f34039ff130f224ba3462d670a1d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotateTo-request)))
  "Returns md5sum for a message object of type 'RotateTo-request"
  "e91f34039ff130f224ba3462d670a1d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotateTo-request>)))
  "Returns full string definition for message of type '<RotateTo-request>"
  (cl:format cl:nil "float32 roll~%float32 pitch~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotateTo-request)))
  "Returns full string definition for message of type 'RotateTo-request"
  (cl:format cl:nil "float32 roll~%float32 pitch~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotateTo-request>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotateTo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RotateTo-request
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude RotateTo-response.msg.html

(cl:defclass <RotateTo-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RotateTo-response (<RotateTo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RotateTo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RotateTo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<RotateTo-response> is deprecated: use maneuver-srv:RotateTo-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <RotateTo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RotateTo-response>) ostream)
  "Serializes a message object of type '<RotateTo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RotateTo-response>) istream)
  "Deserializes a message object of type '<RotateTo-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RotateTo-response>)))
  "Returns string type for a service object of type '<RotateTo-response>"
  "maneuver/RotateToResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateTo-response)))
  "Returns string type for a service object of type 'RotateTo-response"
  "maneuver/RotateToResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RotateTo-response>)))
  "Returns md5sum for a message object of type '<RotateTo-response>"
  "e91f34039ff130f224ba3462d670a1d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RotateTo-response)))
  "Returns md5sum for a message object of type 'RotateTo-response"
  "e91f34039ff130f224ba3462d670a1d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RotateTo-response>)))
  "Returns full string definition for message of type '<RotateTo-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RotateTo-response)))
  "Returns full string definition for message of type 'RotateTo-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RotateTo-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RotateTo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RotateTo-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RotateTo)))
  'RotateTo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RotateTo)))
  'RotateTo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RotateTo)))
  "Returns string type for a service object of type '<RotateTo>"
  "maneuver/RotateTo")