;; Auto-generated. Do not edit!


(when (boundp 'omni_firmware::FullPose)
  (if (not (find-package "OMNI_FIRMWARE"))
    (make-package "OMNI_FIRMWARE"))
  (shadow 'FullPose (find-package "OMNI_FIRMWARE")))
(unless (find-package "OMNI_FIRMWARE::FULLPOSE")
  (make-package "OMNI_FIRMWARE::FULLPOSE"))

(in-package "ROS")
;;//! \htmlinclude FullPose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass omni_firmware::FullPose
  :super ros::object
  :slots (_header _pose _vel _acc ))

(defmethod omni_firmware::FullPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:vel __vel) (instance geometry_msgs::Twist :init))
    ((:acc __acc) (instance geometry_msgs::Accel :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _vel __vel)
   (setq _acc __acc)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:vel
   (&rest __vel)
   (if (keywordp (car __vel))
       (send* _vel __vel)
     (progn
       (if __vel (setq _vel (car __vel)))
       _vel)))
  (:acc
   (&rest __acc)
   (if (keywordp (car __acc))
       (send* _acc __acc)
     (progn
       (if __acc (setq _acc (car __acc)))
       _acc)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _vel
    (send _vel :serialization-length)
    ;; geometry_msgs/Accel _acc
    (send _acc :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _vel
       (send _vel :serialize s)
     ;; geometry_msgs/Accel _acc
       (send _acc :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;; geometry_msgs/Accel _acc
     (send _acc :deserialize buf ptr-) (incf ptr- (send _acc :serialization-length))
   ;;
   self)
  )

(setf (get omni_firmware::FullPose :md5sum-) "273d5e9fd6a8baae8139010997274f3b")
(setf (get omni_firmware::FullPose :datatype-) "omni_firmware/FullPose")
(setf (get omni_firmware::FullPose :definition-)
      "std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist vel
geometry_msgs/Accel acc

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :omni_firmware/FullPose "273d5e9fd6a8baae8139010997274f3b")


