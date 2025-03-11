; Auto-generated. Do not edit!


(cl:in-package servo_control_node-srv)


;//! \htmlinclude SetAngles-request.msg.html

(cl:defclass <SetAngles-request> (roslisp-msg-protocol:ros-message)
  ((right
    :reader right
    :initarg :right
    :type cl:integer
    :initform 0)
   (left
    :reader left
    :initarg :left
    :type cl:integer
    :initform 0))
)

(cl:defclass SetAngles-request (<SetAngles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAngles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAngles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_control_node-srv:<SetAngles-request> is deprecated: use servo_control_node-srv:SetAngles-request instead.")))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <SetAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_control_node-srv:right-val is deprecated.  Use servo_control_node-srv:right instead.")
  (right m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <SetAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_control_node-srv:left-val is deprecated.  Use servo_control_node-srv:left instead.")
  (left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAngles-request>) ostream)
  "Serializes a message object of type '<SetAngles-request>"
  (cl:let* ((signed (cl:slot-value msg 'right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAngles-request>) istream)
  "Deserializes a message object of type '<SetAngles-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAngles-request>)))
  "Returns string type for a service object of type '<SetAngles-request>"
  "servo_control_node/SetAnglesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAngles-request)))
  "Returns string type for a service object of type 'SetAngles-request"
  "servo_control_node/SetAnglesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAngles-request>)))
  "Returns md5sum for a message object of type '<SetAngles-request>"
  "af6660be3609fe9894731472db7c883b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAngles-request)))
  "Returns md5sum for a message object of type 'SetAngles-request"
  "af6660be3609fe9894731472db7c883b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAngles-request>)))
  "Returns full string definition for message of type '<SetAngles-request>"
  (cl:format cl:nil "# srv/SetAngles.srv~%~%int32 right   # Angle for the right motor~%int32 left    # Angle for the left motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAngles-request)))
  "Returns full string definition for message of type 'SetAngles-request"
  (cl:format cl:nil "# srv/SetAngles.srv~%~%int32 right   # Angle for the right motor~%int32 left    # Angle for the left motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAngles-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAngles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAngles-request
    (cl:cons ':right (right msg))
    (cl:cons ':left (left msg))
))
;//! \htmlinclude SetAngles-response.msg.html

(cl:defclass <SetAngles-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetAngles-response (<SetAngles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAngles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAngles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_control_node-srv:<SetAngles-response> is deprecated: use servo_control_node-srv:SetAngles-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_control_node-srv:success-val is deprecated.  Use servo_control_node-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAngles-response>) ostream)
  "Serializes a message object of type '<SetAngles-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAngles-response>) istream)
  "Deserializes a message object of type '<SetAngles-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAngles-response>)))
  "Returns string type for a service object of type '<SetAngles-response>"
  "servo_control_node/SetAnglesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAngles-response)))
  "Returns string type for a service object of type 'SetAngles-response"
  "servo_control_node/SetAnglesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAngles-response>)))
  "Returns md5sum for a message object of type '<SetAngles-response>"
  "af6660be3609fe9894731472db7c883b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAngles-response)))
  "Returns md5sum for a message object of type 'SetAngles-response"
  "af6660be3609fe9894731472db7c883b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAngles-response>)))
  "Returns full string definition for message of type '<SetAngles-response>"
  (cl:format cl:nil "bool success  # Return success if the operation was successful~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAngles-response)))
  "Returns full string definition for message of type 'SetAngles-response"
  (cl:format cl:nil "bool success  # Return success if the operation was successful~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAngles-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAngles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAngles-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAngles)))
  'SetAngles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAngles)))
  'SetAngles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAngles)))
  "Returns string type for a service object of type '<SetAngles>"
  "servo_control_node/SetAngles")