;; Auto-generated. Do not edit!


(when (boundp 'maneuver::ArmDisarm)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'ArmDisarm (find-package "MANEUVER")))
(unless (find-package "MANEUVER::ARMDISARM")
  (make-package "MANEUVER::ARMDISARM"))
(unless (find-package "MANEUVER::ARMDISARMREQUEST")
  (make-package "MANEUVER::ARMDISARMREQUEST"))
(unless (find-package "MANEUVER::ARMDISARMRESPONSE")
  (make-package "MANEUVER::ARMDISARMRESPONSE"))

(in-package "ROS")





(defclass maneuver::ArmDisarmRequest
  :super ros::object
  :slots ())

(defmethod maneuver::ArmDisarmRequest
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

(defclass maneuver::ArmDisarmResponse
  :super ros::object
  :slots (_success ))

(defmethod maneuver::ArmDisarmResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass maneuver::ArmDisarm
  :super ros::object
  :slots ())

(setf (get maneuver::ArmDisarm :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get maneuver::ArmDisarm :datatype-) "maneuver/ArmDisarm")
(setf (get maneuver::ArmDisarm :request) maneuver::ArmDisarmRequest)
(setf (get maneuver::ArmDisarm :response) maneuver::ArmDisarmResponse)

(defmethod maneuver::ArmDisarmRequest
  (:response () (instance maneuver::ArmDisarmResponse :init)))

(setf (get maneuver::ArmDisarmRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get maneuver::ArmDisarmRequest :datatype-) "maneuver/ArmDisarmRequest")
(setf (get maneuver::ArmDisarmRequest :definition-)
      "---
bool success
")

(setf (get maneuver::ArmDisarmResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get maneuver::ArmDisarmResponse :datatype-) "maneuver/ArmDisarmResponse")
(setf (get maneuver::ArmDisarmResponse :definition-)
      "---
bool success
")



(provide :maneuver/ArmDisarm "358e233cde0c8a8bcfea4ce193f8fc15")


