; Auto-generated. Do not edit!


(cl:in-package omni_firmware-msg)


;//! \htmlinclude errorMsg.msg.html

(cl:defclass <errorMsg> (roslisp-msg-protocol:ros-message)
  ((ex
    :reader ex
    :initarg :ex
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (ev
    :reader ev
    :initarg :ev
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (ea
    :reader ea
    :initarg :ea
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (eR
    :reader eR
    :initarg :eR
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (ew
    :reader ew
    :initarg :ew
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (Iex
    :reader Iex
    :initarg :Iex
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (Ier
    :reader Ier
    :initarg :Ier
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (Accd
    :reader Accd
    :initarg :Accd
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (Wrench
    :reader Wrench
    :initarg :Wrench
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (exWrench
    :reader exWrench
    :initarg :exWrench
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (prop
    :reader prop
    :initarg :prop
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass errorMsg (<errorMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <errorMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'errorMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_firmware-msg:<errorMsg> is deprecated: use omni_firmware-msg:errorMsg instead.")))

(cl:ensure-generic-function 'ex-val :lambda-list '(m))
(cl:defmethod ex-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:ex-val is deprecated.  Use omni_firmware-msg:ex instead.")
  (ex m))

(cl:ensure-generic-function 'ev-val :lambda-list '(m))
(cl:defmethod ev-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:ev-val is deprecated.  Use omni_firmware-msg:ev instead.")
  (ev m))

(cl:ensure-generic-function 'ea-val :lambda-list '(m))
(cl:defmethod ea-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:ea-val is deprecated.  Use omni_firmware-msg:ea instead.")
  (ea m))

(cl:ensure-generic-function 'eR-val :lambda-list '(m))
(cl:defmethod eR-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:eR-val is deprecated.  Use omni_firmware-msg:eR instead.")
  (eR m))

(cl:ensure-generic-function 'ew-val :lambda-list '(m))
(cl:defmethod ew-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:ew-val is deprecated.  Use omni_firmware-msg:ew instead.")
  (ew m))

(cl:ensure-generic-function 'Iex-val :lambda-list '(m))
(cl:defmethod Iex-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:Iex-val is deprecated.  Use omni_firmware-msg:Iex instead.")
  (Iex m))

(cl:ensure-generic-function 'Ier-val :lambda-list '(m))
(cl:defmethod Ier-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:Ier-val is deprecated.  Use omni_firmware-msg:Ier instead.")
  (Ier m))

(cl:ensure-generic-function 'Accd-val :lambda-list '(m))
(cl:defmethod Accd-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:Accd-val is deprecated.  Use omni_firmware-msg:Accd instead.")
  (Accd m))

(cl:ensure-generic-function 'Wrench-val :lambda-list '(m))
(cl:defmethod Wrench-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:Wrench-val is deprecated.  Use omni_firmware-msg:Wrench instead.")
  (Wrench m))

(cl:ensure-generic-function 'exWrench-val :lambda-list '(m))
(cl:defmethod exWrench-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:exWrench-val is deprecated.  Use omni_firmware-msg:exWrench instead.")
  (exWrench m))

(cl:ensure-generic-function 'prop-val :lambda-list '(m))
(cl:defmethod prop-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:prop-val is deprecated.  Use omni_firmware-msg:prop instead.")
  (prop m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <errorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:weight-val is deprecated.  Use omni_firmware-msg:weight instead.")
  (weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <errorMsg>) ostream)
  "Serializes a message object of type '<errorMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ex) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ev) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ea) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'eR) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ew) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Iex) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Ier) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Accd) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Wrench))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Wrench))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'exWrench))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'exWrench))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'prop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'prop))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <errorMsg>) istream)
  "Deserializes a message object of type '<errorMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ex) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ev) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ea) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'eR) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ew) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Iex) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Ier) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Accd) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Wrench) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Wrench)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'exWrench) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'exWrench)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'prop) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'prop)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<errorMsg>)))
  "Returns string type for a message object of type '<errorMsg>"
  "omni_firmware/errorMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'errorMsg)))
  "Returns string type for a message object of type 'errorMsg"
  "omni_firmware/errorMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<errorMsg>)))
  "Returns md5sum for a message object of type '<errorMsg>"
  "aca7c5154e5df4742f8034562501f6c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'errorMsg)))
  "Returns md5sum for a message object of type 'errorMsg"
  "aca7c5154e5df4742f8034562501f6c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<errorMsg>)))
  "Returns full string definition for message of type '<errorMsg>"
  (cl:format cl:nil "geometry_msgs/Vector3 ex~%geometry_msgs/Vector3 ev~%geometry_msgs/Vector3 ea~%geometry_msgs/Vector3 eR~%geometry_msgs/Vector3 ew~%geometry_msgs/Vector3 Iex~%geometry_msgs/Vector3 Ier~%geometry_msgs/Vector3 Accd~%float32[] Wrench~%float32[] exWrench~%float32[] prop~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'errorMsg)))
  "Returns full string definition for message of type 'errorMsg"
  (cl:format cl:nil "geometry_msgs/Vector3 ex~%geometry_msgs/Vector3 ev~%geometry_msgs/Vector3 ea~%geometry_msgs/Vector3 eR~%geometry_msgs/Vector3 ew~%geometry_msgs/Vector3 Iex~%geometry_msgs/Vector3 Ier~%geometry_msgs/Vector3 Accd~%float32[] Wrench~%float32[] exWrench~%float32[] prop~%float32 weight~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <errorMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ex))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ev))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ea))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'eR))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ew))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Iex))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Ier))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Accd))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Wrench) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exWrench) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'prop) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <errorMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'errorMsg
    (cl:cons ':ex (ex msg))
    (cl:cons ':ev (ev msg))
    (cl:cons ':ea (ea msg))
    (cl:cons ':eR (eR msg))
    (cl:cons ':ew (ew msg))
    (cl:cons ':Iex (Iex msg))
    (cl:cons ':Ier (Ier msg))
    (cl:cons ':Accd (Accd msg))
    (cl:cons ':Wrench (Wrench msg))
    (cl:cons ':exWrench (exWrench msg))
    (cl:cons ':prop (prop msg))
    (cl:cons ':weight (weight msg))
))
