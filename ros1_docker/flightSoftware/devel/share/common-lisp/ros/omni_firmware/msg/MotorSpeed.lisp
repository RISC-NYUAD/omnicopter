; Auto-generated. Do not edit!


(cl:in-package omni_firmware-msg)


;//! \htmlinclude MotorSpeed.msg.html

(cl:defclass <MotorSpeed> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (velocity
    :reader velocity
    :initarg :velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MotorSpeed (<MotorSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_firmware-msg:<MotorSpeed> is deprecated: use omni_firmware-msg:MotorSpeed instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MotorSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:name-val is deprecated.  Use omni_firmware-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <MotorSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:velocity-val is deprecated.  Use omni_firmware-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSpeed>) ostream)
  "Serializes a message object of type '<MotorSpeed>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'velocity))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSpeed>) istream)
  "Deserializes a message object of type '<MotorSpeed>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'velocity) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'velocity)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSpeed>)))
  "Returns string type for a message object of type '<MotorSpeed>"
  "omni_firmware/MotorSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSpeed)))
  "Returns string type for a message object of type 'MotorSpeed"
  "omni_firmware/MotorSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSpeed>)))
  "Returns md5sum for a message object of type '<MotorSpeed>"
  "f1d54c3c8a939a204eb96629420d7ab7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSpeed)))
  "Returns md5sum for a message object of type 'MotorSpeed"
  "f1d54c3c8a939a204eb96629420d7ab7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSpeed>)))
  "Returns full string definition for message of type '<MotorSpeed>"
  (cl:format cl:nil "string[] name~%float32[] velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSpeed)))
  "Returns full string definition for message of type 'MotorSpeed"
  (cl:format cl:nil "string[] name~%float32[] velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSpeed>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSpeed
    (cl:cons ':name (name msg))
    (cl:cons ':velocity (velocity msg))
))
