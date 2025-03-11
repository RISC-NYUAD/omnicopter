; Auto-generated. Do not edit!


(cl:in-package omni_firmware-msg)


;//! \htmlinclude FullPose.msg.html

(cl:defclass <FullPose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (acc
    :reader acc
    :initarg :acc
    :type geometry_msgs-msg:Accel
    :initform (cl:make-instance 'geometry_msgs-msg:Accel)))
)

(cl:defclass FullPose (<FullPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name omni_firmware-msg:<FullPose> is deprecated: use omni_firmware-msg:FullPose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FullPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:header-val is deprecated.  Use omni_firmware-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <FullPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:pose-val is deprecated.  Use omni_firmware-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <FullPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:vel-val is deprecated.  Use omni_firmware-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <FullPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader omni_firmware-msg:acc-val is deprecated.  Use omni_firmware-msg:acc instead.")
  (acc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullPose>) ostream)
  "Serializes a message object of type '<FullPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullPose>) istream)
  "Deserializes a message object of type '<FullPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullPose>)))
  "Returns string type for a message object of type '<FullPose>"
  "omni_firmware/FullPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullPose)))
  "Returns string type for a message object of type 'FullPose"
  "omni_firmware/FullPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullPose>)))
  "Returns md5sum for a message object of type '<FullPose>"
  "273d5e9fd6a8baae8139010997274f3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullPose)))
  "Returns md5sum for a message object of type 'FullPose"
  "273d5e9fd6a8baae8139010997274f3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullPose>)))
  "Returns full string definition for message of type '<FullPose>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist vel~%geometry_msgs/Accel acc~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullPose)))
  "Returns full string definition for message of type 'FullPose"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist vel~%geometry_msgs/Accel acc~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullPose>))
  "Converts a ROS message object to a list"
  (cl:list 'FullPose
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
))
