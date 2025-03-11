; Auto-generated. Do not edit!


(cl:in-package omni_firmware-srv)


;//! \htmlinclude ZeroUKF-request.msg.html

(cl:defclass <ZeroUKF-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ZeroUKF-request (<ZeroUKF-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZeroUKF-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZeroUKF-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_firmware-srv:<ZeroUKF-request> is deprecated: use omni_firmware-srv:ZeroUKF-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZeroUKF-request>) ostream)
  "Serializes a message object of type '<ZeroUKF-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZeroUKF-request>) istream)
  "Deserializes a message object of type '<ZeroUKF-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZeroUKF-request>)))
  "Returns string type for a service object of type '<ZeroUKF-request>"
  "omni_firmware/ZeroUKFRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZeroUKF-request)))
  "Returns string type for a service object of type 'ZeroUKF-request"
  "omni_firmware/ZeroUKFRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZeroUKF-request>)))
  "Returns md5sum for a message object of type '<ZeroUKF-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZeroUKF-request)))
  "Returns md5sum for a message object of type 'ZeroUKF-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZeroUKF-request>)))
  "Returns full string definition for message of type '<ZeroUKF-request>"
  (cl:format cl:nil "# ZeroRotation.srv~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZeroUKF-request)))
  "Returns full string definition for message of type 'ZeroUKF-request"
  (cl:format cl:nil "# ZeroRotation.srv~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZeroUKF-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZeroUKF-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ZeroUKF-request
))
;//! \htmlinclude ZeroUKF-response.msg.html

(cl:defclass <ZeroUKF-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ZeroUKF-response (<ZeroUKF-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZeroUKF-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZeroUKF-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_firmware-srv:<ZeroUKF-response> is deprecated: use omni_firmware-srv:ZeroUKF-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ZeroUKF-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-srv:success-val is deprecated.  Use omni_firmware-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZeroUKF-response>) ostream)
  "Serializes a message object of type '<ZeroUKF-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZeroUKF-response>) istream)
  "Deserializes a message object of type '<ZeroUKF-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZeroUKF-response>)))
  "Returns string type for a service object of type '<ZeroUKF-response>"
  "omni_firmware/ZeroUKFResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZeroUKF-response)))
  "Returns string type for a service object of type 'ZeroUKF-response"
  "omni_firmware/ZeroUKFResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZeroUKF-response>)))
  "Returns md5sum for a message object of type '<ZeroUKF-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZeroUKF-response)))
  "Returns md5sum for a message object of type 'ZeroUKF-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZeroUKF-response>)))
  "Returns full string definition for message of type '<ZeroUKF-response>"
  (cl:format cl:nil "bool success  # indicates whether the operation was successful~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZeroUKF-response)))
  "Returns full string definition for message of type 'ZeroUKF-response"
  (cl:format cl:nil "bool success  # indicates whether the operation was successful~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZeroUKF-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZeroUKF-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ZeroUKF-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ZeroUKF)))
  'ZeroUKF-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ZeroUKF)))
  'ZeroUKF-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZeroUKF)))
  "Returns string type for a service object of type '<ZeroUKF>"
  "omni_firmware/ZeroUKF")