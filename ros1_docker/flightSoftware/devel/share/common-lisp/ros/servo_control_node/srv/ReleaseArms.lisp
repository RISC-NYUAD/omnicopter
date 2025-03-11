; Auto-generated. Do not edit!


(cl:in-package servo_control_node-srv)


;//! \htmlinclude ReleaseArms-request.msg.html

(cl:defclass <ReleaseArms-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReleaseArms-request (<ReleaseArms-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseArms-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseArms-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_control_node-srv:<ReleaseArms-request> is deprecated: use servo_control_node-srv:ReleaseArms-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseArms-request>) ostream)
  "Serializes a message object of type '<ReleaseArms-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseArms-request>) istream)
  "Deserializes a message object of type '<ReleaseArms-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseArms-request>)))
  "Returns string type for a service object of type '<ReleaseArms-request>"
  "servo_control_node/ReleaseArmsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseArms-request)))
  "Returns string type for a service object of type 'ReleaseArms-request"
  "servo_control_node/ReleaseArmsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseArms-request>)))
  "Returns md5sum for a message object of type '<ReleaseArms-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseArms-request)))
  "Returns md5sum for a message object of type 'ReleaseArms-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseArms-request>)))
  "Returns full string definition for message of type '<ReleaseArms-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseArms-request)))
  "Returns full string definition for message of type 'ReleaseArms-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseArms-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseArms-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseArms-request
))
;//! \htmlinclude ReleaseArms-response.msg.html

(cl:defclass <ReleaseArms-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReleaseArms-response (<ReleaseArms-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseArms-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseArms-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_control_node-srv:<ReleaseArms-response> is deprecated: use servo_control_node-srv:ReleaseArms-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseArms-response>) ostream)
  "Serializes a message object of type '<ReleaseArms-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseArms-response>) istream)
  "Deserializes a message object of type '<ReleaseArms-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseArms-response>)))
  "Returns string type for a service object of type '<ReleaseArms-response>"
  "servo_control_node/ReleaseArmsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseArms-response)))
  "Returns string type for a service object of type 'ReleaseArms-response"
  "servo_control_node/ReleaseArmsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseArms-response>)))
  "Returns md5sum for a message object of type '<ReleaseArms-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseArms-response)))
  "Returns md5sum for a message object of type 'ReleaseArms-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseArms-response>)))
  "Returns full string definition for message of type '<ReleaseArms-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseArms-response)))
  "Returns full string definition for message of type 'ReleaseArms-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseArms-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseArms-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseArms-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReleaseArms)))
  'ReleaseArms-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReleaseArms)))
  'ReleaseArms-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseArms)))
  "Returns string type for a service object of type '<ReleaseArms>"
  "servo_control_node/ReleaseArms")