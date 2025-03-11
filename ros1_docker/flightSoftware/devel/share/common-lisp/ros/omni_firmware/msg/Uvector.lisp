; Auto-generated. Do not edit!


(cl:in-package omni_firmware-msg)


;//! \htmlinclude Uvector.msg.html

(cl:defclass <Uvector> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Uvector (<Uvector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Uvector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Uvector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_firmware-msg:<Uvector> is deprecated: use omni_firmware-msg:Uvector instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Uvector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:value-val is deprecated.  Use omni_firmware-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Uvector>) ostream)
  "Serializes a message object of type '<Uvector>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Uvector>) istream)
  "Deserializes a message object of type '<Uvector>"
  (cl:setf (cl:slot-value msg 'value) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'value)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Uvector>)))
  "Returns string type for a message object of type '<Uvector>"
  "omni_firmware/Uvector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Uvector)))
  "Returns string type for a message object of type 'Uvector"
  "omni_firmware/Uvector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Uvector>)))
  "Returns md5sum for a message object of type '<Uvector>"
  "3cac8a89b50e7d740ace6fafac0a09de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Uvector)))
  "Returns md5sum for a message object of type 'Uvector"
  "3cac8a89b50e7d740ace6fafac0a09de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Uvector>)))
  "Returns full string definition for message of type '<Uvector>"
  (cl:format cl:nil "float32[12] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Uvector)))
  "Returns full string definition for message of type 'Uvector"
  (cl:format cl:nil "float32[12] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Uvector>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Uvector>))
  "Converts a ROS message object to a list"
  (cl:list 'Uvector
    (cl:cons ':value (value msg))
))
