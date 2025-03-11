;; Auto-generated. Do not edit!


(when (boundp 'servo_control_node::ReleaseArms)
  (if (not (find-package "SERVO_CONTROL_NODE"))
    (make-package "SERVO_CONTROL_NODE"))
  (shadow 'ReleaseArms (find-package "SERVO_CONTROL_NODE")))
(unless (find-package "SERVO_CONTROL_NODE::RELEASEARMS")
  (make-package "SERVO_CONTROL_NODE::RELEASEARMS"))
(unless (find-package "SERVO_CONTROL_NODE::RELEASEARMSREQUEST")
  (make-package "SERVO_CONTROL_NODE::RELEASEARMSREQUEST"))
(unless (find-package "SERVO_CONTROL_NODE::RELEASEARMSRESPONSE")
  (make-package "SERVO_CONTROL_NODE::RELEASEARMSRESPONSE"))

(in-package "ROS")





(defclass servo_control_node::ReleaseArmsRequest
  :super ros::object
  :slots ())

(defmethod servo_control_node::ReleaseArmsRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass servo_control_node::ReleaseArmsResponse
  :super ros::object
  :slots ())

(defmethod servo_control_node::ReleaseArmsResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass servo_control_node::ReleaseArms
  :super ros::object
  :slots ())

(setf (get servo_control_node::ReleaseArms :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get servo_control_node::ReleaseArms :datatype-) "servo_control_node/ReleaseArms")
(setf (get servo_control_node::ReleaseArms :request) servo_control_node::ReleaseArmsRequest)
(setf (get servo_control_node::ReleaseArms :response) servo_control_node::ReleaseArmsResponse)

(defmethod servo_control_node::ReleaseArmsRequest
  (:response () (instance servo_control_node::ReleaseArmsResponse :init)))

(setf (get servo_control_node::ReleaseArmsRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get servo_control_node::ReleaseArmsRequest :datatype-) "servo_control_node/ReleaseArmsRequest")
(setf (get servo_control_node::ReleaseArmsRequest :definition-)
      "
---
")

(setf (get servo_control_node::ReleaseArmsResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get servo_control_node::ReleaseArmsResponse :datatype-) "servo_control_node/ReleaseArmsResponse")
(setf (get servo_control_node::ReleaseArmsResponse :definition-)
      "
---
")



(provide :servo_control_node/ReleaseArms "d41d8cd98f00b204e9800998ecf8427e")


