; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude FullFlip-request.msg.html

(cl:defclass <FullFlip-request> (roslisp-msg-protocol:ros-message)
  ((roll_bool
    :reader roll_bool
    :initarg :roll_bool
    :type cl:boolean
    :initform cl:nil)
   (pitch_bool
    :reader pitch_bool
    :initarg :pitch_bool
    :type cl:boolean
    :initform cl:nil)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass FullFlip-request (<FullFlip-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullFlip-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullFlip-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<FullFlip-request> is deprecated: use maneuver-srv:FullFlip-request instead.")))

(cl:ensure-generic-function 'roll_bool-val :lambda-list '(m))
(cl:defmethod roll_bool-val ((m <FullFlip-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:roll_bool-val is deprecated.  Use maneuver-srv:roll_bool instead.")
  (roll_bool m))

(cl:ensure-generic-function 'pitch_bool-val :lambda-list '(m))
(cl:defmethod pitch_bool-val ((m <FullFlip-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:pitch_bool-val is deprecated.  Use maneuver-srv:pitch_bool instead.")
  (pitch_bool m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <FullFlip-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration-val is deprecated.  Use maneuver-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullFlip-request>) ostream)
  "Serializes a message object of type '<FullFlip-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'roll_bool) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pitch_bool) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullFlip-request>) istream)
  "Deserializes a message object of type '<FullFlip-request>"
    (cl:setf (cl:slot-value msg 'roll_bool) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pitch_bool) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullFlip-request>)))
  "Returns string type for a service object of type '<FullFlip-request>"
  "maneuver/FullFlipRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullFlip-request)))
  "Returns string type for a service object of type 'FullFlip-request"
  "maneuver/FullFlipRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullFlip-request>)))
  "Returns md5sum for a message object of type '<FullFlip-request>"
  "432212f34386b2b3d536658964c298f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullFlip-request)))
  "Returns md5sum for a message object of type 'FullFlip-request"
  "432212f34386b2b3d536658964c298f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullFlip-request>)))
  "Returns full string definition for message of type '<FullFlip-request>"
  (cl:format cl:nil "bool roll_bool~%bool pitch_bool~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullFlip-request)))
  "Returns full string definition for message of type 'FullFlip-request"
  (cl:format cl:nil "bool roll_bool~%bool pitch_bool~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullFlip-request>))
  (cl:+ 0
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullFlip-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FullFlip-request
    (cl:cons ':roll_bool (roll_bool msg))
    (cl:cons ':pitch_bool (pitch_bool msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude FullFlip-response.msg.html

(cl:defclass <FullFlip-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FullFlip-response (<FullFlip-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullFlip-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullFlip-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<FullFlip-response> is deprecated: use maneuver-srv:FullFlip-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <FullFlip-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullFlip-response>) ostream)
  "Serializes a message object of type '<FullFlip-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullFlip-response>) istream)
  "Deserializes a message object of type '<FullFlip-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullFlip-response>)))
  "Returns string type for a service object of type '<FullFlip-response>"
  "maneuver/FullFlipResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullFlip-response)))
  "Returns string type for a service object of type 'FullFlip-response"
  "maneuver/FullFlipResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullFlip-response>)))
  "Returns md5sum for a message object of type '<FullFlip-response>"
  "432212f34386b2b3d536658964c298f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullFlip-response)))
  "Returns md5sum for a message object of type 'FullFlip-response"
  "432212f34386b2b3d536658964c298f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullFlip-response>)))
  "Returns full string definition for message of type '<FullFlip-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullFlip-response)))
  "Returns full string definition for message of type 'FullFlip-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullFlip-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullFlip-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FullFlip-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FullFlip)))
  'FullFlip-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FullFlip)))
  'FullFlip-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullFlip)))
  "Returns string type for a service object of type '<FullFlip>"
  "maneuver/FullFlip")