;; Auto-generated. Do not edit!


(when (boundp 'omni_firmware::ZeroUKF)
  (if (not (find-package "OMNI_FIRMWARE"))
    (make-package "OMNI_FIRMWARE"))
  (shadow 'ZeroUKF (find-package "OMNI_FIRMWARE")))
(unless (find-package "OMNI_FIRMWARE::ZEROUKF")
  (make-package "OMNI_FIRMWARE::ZEROUKF"))
(unless (find-package "OMNI_FIRMWARE::ZEROUKFREQUEST")
  (make-package "OMNI_FIRMWARE::ZEROUKFREQUEST"))
(unless (find-package "OMNI_FIRMWARE::ZEROUKFRESPONSE")
  (make-package "OMNI_FIRMWARE::ZEROUKFRESPONSE"))

(in-package "ROS")





(defclass omni_firmware::ZeroUKFRequest
  :super ros::object
  :slots ())

(defmethod omni_firmware::ZeroUKFRequest
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

(defclass omni_firmware::ZeroUKFResponse
  :super ros::object
  :slots (_success ))

(defmethod omni_firmware::ZeroUKFResponse
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

(defclass omni_firmware::ZeroUKF
  :super ros::object
  :slots ())

(setf (get omni_firmware::ZeroUKF :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get omni_firmware::ZeroUKF :datatype-) "omni_firmware/ZeroUKF")
(setf (get omni_firmware::ZeroUKF :request) omni_firmware::ZeroUKFRequest)
(setf (get omni_firmware::ZeroUKF :response) omni_firmware::ZeroUKFResponse)

(defmethod omni_firmware::ZeroUKFRequest
  (:response () (instance omni_firmware::ZeroUKFResponse :init)))

(setf (get omni_firmware::ZeroUKFRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get omni_firmware::ZeroUKFRequest :datatype-) "omni_firmware/ZeroUKFRequest")
(setf (get omni_firmware::ZeroUKFRequest :definition-)
      "# ZeroRotation.srv
---
bool success  # indicates whether the operation was successful

")

(setf (get omni_firmware::ZeroUKFResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get omni_firmware::ZeroUKFResponse :datatype-) "omni_firmware/ZeroUKFResponse")
(setf (get omni_firmware::ZeroUKFResponse :definition-)
      "# ZeroRotation.srv
---
bool success  # indicates whether the operation was successful

")



(provide :omni_firmware/ZeroUKF "358e233cde0c8a8bcfea4ce193f8fc15")


