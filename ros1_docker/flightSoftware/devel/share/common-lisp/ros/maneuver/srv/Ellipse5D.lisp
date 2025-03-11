; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude Ellipse5D-request.msg.html

(cl:defclass <Ellipse5D-request> (roslisp-msg-protocol:ros-message)
  ((x_min
    :reader x_min
    :initarg :x_min
    :type cl:float
    :initform 0.0)
   (x_max
    :reader x_max
    :initarg :x_max
    :type cl:float
    :initform 0.0)
   (y_min
    :reader y_min
    :initarg :y_min
    :type cl:float
    :initform 0.0)
   (y_max
    :reader y_max
    :initarg :y_max
    :type cl:float
    :initform 0.0)
   (z_min
    :reader z_min
    :initarg :z_min
    :type cl:float
    :initform 0.0)
   (z_max
    :reader z_max
    :initarg :z_max
    :type cl:float
    :initform 0.0)
   (roll_start
    :reader roll_start
    :initarg :roll_start
    :type cl:float
    :initform 0.0)
   (roll_mid
    :reader roll_mid
    :initarg :roll_mid
    :type cl:float
    :initform 0.0)
   (roll_end
    :reader roll_end
    :initarg :roll_end
    :type cl:float
    :initform 0.0)
   (pitch_start
    :reader pitch_start
    :initarg :pitch_start
    :type cl:float
    :initform 0.0)
   (pitch_mid
    :reader pitch_mid
    :initarg :pitch_mid
    :type cl:float
    :initform 0.0)
   (pitch_end
    :reader pitch_end
    :initarg :pitch_end
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

(cl:defclass Ellipse5D-request (<Ellipse5D-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ellipse5D-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ellipse5D-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<Ellipse5D-request> is deprecated: use maneuver-srv:Ellipse5D-request instead.")))

(cl:ensure-generic-function 'x_min-val :lambda-list '(m))
(cl:defmethod x_min-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:x_min-val is deprecated.  Use maneuver-srv:x_min instead.")
  (x_min m))

(cl:ensure-generic-function 'x_max-val :lambda-list '(m))
(cl:defmethod x_max-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:x_max-val is deprecated.  Use maneuver-srv:x_max instead.")
  (x_max m))

(cl:ensure-generic-function 'y_min-val :lambda-list '(m))
(cl:defmethod y_min-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:y_min-val is deprecated.  Use maneuver-srv:y_min instead.")
  (y_min m))

(cl:ensure-generic-function 'y_max-val :lambda-list '(m))
(cl:defmethod y_max-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:y_max-val is deprecated.  Use maneuver-srv:y_max instead.")
  (y_max m))

(cl:ensure-generic-function 'z_min-val :lambda-list '(m))
(cl:defmethod z_min-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:z_min-val is deprecated.  Use maneuver-srv:z_min instead.")
  (z_min m))

(cl:ensure-generic-function 'z_max-val :lambda-list '(m))
(cl:defmethod z_max-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:z_max-val is deprecated.  Use maneuver-srv:z_max instead.")
  (z_max m))

(cl:ensure-generic-function 'roll_start-val :lambda-list '(m))
(cl:defmethod roll_start-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:roll_start-val is deprecated.  Use maneuver-srv:roll_start instead.")
  (roll_start m))

(cl:ensure-generic-function 'roll_mid-val :lambda-list '(m))
(cl:defmethod roll_mid-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:roll_mid-val is deprecated.  Use maneuver-srv:roll_mid instead.")
  (roll_mid m))

(cl:ensure-generic-function 'roll_end-val :lambda-list '(m))
(cl:defmethod roll_end-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:roll_end-val is deprecated.  Use maneuver-srv:roll_end instead.")
  (roll_end m))

(cl:ensure-generic-function 'pitch_start-val :lambda-list '(m))
(cl:defmethod pitch_start-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:pitch_start-val is deprecated.  Use maneuver-srv:pitch_start instead.")
  (pitch_start m))

(cl:ensure-generic-function 'pitch_mid-val :lambda-list '(m))
(cl:defmethod pitch_mid-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:pitch_mid-val is deprecated.  Use maneuver-srv:pitch_mid instead.")
  (pitch_mid m))

(cl:ensure-generic-function 'pitch_end-val :lambda-list '(m))
(cl:defmethod pitch_end-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:pitch_end-val is deprecated.  Use maneuver-srv:pitch_end instead.")
  (pitch_end m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:yaw-val is deprecated.  Use maneuver-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Ellipse5D-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:duration-val is deprecated.  Use maneuver-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ellipse5D-request>) ostream)
  "Serializes a message object of type '<Ellipse5D-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_mid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_end))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_mid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_end))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ellipse5D-request>) istream)
  "Deserializes a message object of type '<Ellipse5D-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_start) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_mid) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_end) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_start) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_mid) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_end) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ellipse5D-request>)))
  "Returns string type for a service object of type '<Ellipse5D-request>"
  "maneuver/Ellipse5DRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ellipse5D-request)))
  "Returns string type for a service object of type 'Ellipse5D-request"
  "maneuver/Ellipse5DRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ellipse5D-request>)))
  "Returns md5sum for a message object of type '<Ellipse5D-request>"
  "bdb0ce274c1da7c8239ea3857b021bbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ellipse5D-request)))
  "Returns md5sum for a message object of type 'Ellipse5D-request"
  "bdb0ce274c1da7c8239ea3857b021bbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ellipse5D-request>)))
  "Returns full string definition for message of type '<Ellipse5D-request>"
  (cl:format cl:nil "float32 x_min~%float32 x_max~%float32 y_min~%float32 y_max~%float32 z_min~%float32 z_max~%float32 roll_start~%float32 roll_mid~%float32 roll_end~%float32 pitch_start~%float32 pitch_mid~%float32 pitch_end~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ellipse5D-request)))
  "Returns full string definition for message of type 'Ellipse5D-request"
  (cl:format cl:nil "float32 x_min~%float32 x_max~%float32 y_min~%float32 y_max~%float32 z_min~%float32 z_max~%float32 roll_start~%float32 roll_mid~%float32 roll_end~%float32 pitch_start~%float32 pitch_mid~%float32 pitch_end~%float32 yaw~%float32 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ellipse5D-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ellipse5D-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Ellipse5D-request
    (cl:cons ':x_min (x_min msg))
    (cl:cons ':x_max (x_max msg))
    (cl:cons ':y_min (y_min msg))
    (cl:cons ':y_max (y_max msg))
    (cl:cons ':z_min (z_min msg))
    (cl:cons ':z_max (z_max msg))
    (cl:cons ':roll_start (roll_start msg))
    (cl:cons ':roll_mid (roll_mid msg))
    (cl:cons ':roll_end (roll_end msg))
    (cl:cons ':pitch_start (pitch_start msg))
    (cl:cons ':pitch_mid (pitch_mid msg))
    (cl:cons ':pitch_end (pitch_end msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude Ellipse5D-response.msg.html

(cl:defclass <Ellipse5D-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Ellipse5D-response (<Ellipse5D-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ellipse5D-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ellipse5D-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<Ellipse5D-response> is deprecated: use maneuver-srv:Ellipse5D-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Ellipse5D-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:status-val is deprecated.  Use maneuver-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ellipse5D-response>) ostream)
  "Serializes a message object of type '<Ellipse5D-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ellipse5D-response>) istream)
  "Deserializes a message object of type '<Ellipse5D-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ellipse5D-response>)))
  "Returns string type for a service object of type '<Ellipse5D-response>"
  "maneuver/Ellipse5DResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ellipse5D-response)))
  "Returns string type for a service object of type 'Ellipse5D-response"
  "maneuver/Ellipse5DResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ellipse5D-response>)))
  "Returns md5sum for a message object of type '<Ellipse5D-response>"
  "bdb0ce274c1da7c8239ea3857b021bbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ellipse5D-response)))
  "Returns md5sum for a message object of type 'Ellipse5D-response"
  "bdb0ce274c1da7c8239ea3857b021bbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ellipse5D-response>)))
  "Returns full string definition for message of type '<Ellipse5D-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ellipse5D-response)))
  "Returns full string definition for message of type 'Ellipse5D-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ellipse5D-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ellipse5D-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Ellipse5D-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Ellipse5D)))
  'Ellipse5D-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Ellipse5D)))
  'Ellipse5D-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ellipse5D)))
  "Returns string type for a service object of type '<Ellipse5D>"
  "maneuver/Ellipse5D")