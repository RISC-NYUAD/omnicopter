; Auto-generated. Do not edit!


(cl:in-package maneuver-srv)


;//! \htmlinclude ArmDisarm-request.msg.html

(cl:defclass <ArmDisarm-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ArmDisarm-request (<ArmDisarm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmDisarm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmDisarm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<ArmDisarm-request> is deprecated: use maneuver-srv:ArmDisarm-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmDisarm-request>) ostream)
  "Serializes a message object of type '<ArmDisarm-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmDisarm-request>) istream)
  "Deserializes a message object of type '<ArmDisarm-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmDisarm-request>)))
  "Returns string type for a service object of type '<ArmDisarm-request>"
  "maneuver/ArmDisarmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmDisarm-request)))
  "Returns string type for a service object of type 'ArmDisarm-request"
  "maneuver/ArmDisarmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmDisarm-request>)))
  "Returns md5sum for a message object of type '<ArmDisarm-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmDisarm-request)))
  "Returns md5sum for a message object of type 'ArmDisarm-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmDisarm-request>)))
  "Returns full string definition for message of type '<ArmDisarm-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmDisarm-request)))
  "Returns full string definition for message of type 'ArmDisarm-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmDisarm-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmDisarm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmDisarm-request
))
;//! \htmlinclude ArmDisarm-response.msg.html

(cl:defclass <ArmDisarm-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArmDisarm-response (<ArmDisarm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmDisarm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmDisarm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name maneuver-srv:<ArmDisarm-response> is deprecated: use maneuver-srv:ArmDisarm-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ArmDisarm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader maneuver-srv:success-val is deprecated.  Use maneuver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmDisarm-response>) ostream)
  "Serializes a message object of type '<ArmDisarm-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmDisarm-response>) istream)
  "Deserializes a message object of type '<ArmDisarm-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmDisarm-response>)))
  "Returns string type for a service object of type '<ArmDisarm-response>"
  "maneuver/ArmDisarmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmDisarm-response)))
  "Returns string type for a service object of type 'ArmDisarm-response"
  "maneuver/ArmDisarmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmDisarm-response>)))
  "Returns md5sum for a message object of type '<ArmDisarm-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmDisarm-response)))
  "Returns md5sum for a message object of type 'ArmDisarm-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmDisarm-response>)))
  "Returns full string definition for message of type '<ArmDisarm-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmDisarm-response)))
  "Returns full string definition for message of type 'ArmDisarm-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmDisarm-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmDisarm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmDisarm-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ArmDisarm)))
  'ArmDisarm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ArmDisarm)))
  'ArmDisarm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmDisarm)))
  "Returns string type for a service object of type '<ArmDisarm>"
  "maneuver/ArmDisarm")