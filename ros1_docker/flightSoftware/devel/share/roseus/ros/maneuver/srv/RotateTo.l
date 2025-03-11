;; Auto-generated. Do not edit!


(when (boundp 'maneuver::RotateTo)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'RotateTo (find-package "MANEUVER")))
(unless (find-package "MANEUVER::ROTATETO")
  (make-package "MANEUVER::ROTATETO"))
(unless (find-package "MANEUVER::ROTATETOREQUEST")
  (make-package "MANEUVER::ROTATETOREQUEST"))
(unless (find-package "MANEUVER::ROTATETORESPONSE")
  (make-package "MANEUVER::ROTATETORESPONSE"))

(in-package "ROS")





(defclass maneuver::RotateToRequest
  :super ros::object
  :slots (_roll _pitch _yaw _duration ))

(defmethod maneuver::RotateToRequest
  (:init
   (&key
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _duration (float __duration))
   self)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass maneuver::RotateToResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::RotateToResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass maneuver::RotateTo
  :super ros::object
  :slots ())

(setf (get maneuver::RotateTo :md5sum-) "e91f34039ff130f224ba3462d670a1d0")
(setf (get maneuver::RotateTo :datatype-) "maneuver/RotateTo")
(setf (get maneuver::RotateTo :request) maneuver::RotateToRequest)
(setf (get maneuver::RotateTo :response) maneuver::RotateToResponse)

(defmethod maneuver::RotateToRequest
  (:response () (instance maneuver::RotateToResponse :init)))

(setf (get maneuver::RotateToRequest :md5sum-) "e91f34039ff130f224ba3462d670a1d0")
(setf (get maneuver::RotateToRequest :datatype-) "maneuver/RotateToRequest")
(setf (get maneuver::RotateToRequest :definition-)
      "float32 roll
float32 pitch
float32 yaw
float32 duration
---
bool status

")

(setf (get maneuver::RotateToResponse :md5sum-) "e91f34039ff130f224ba3462d670a1d0")
(setf (get maneuver::RotateToResponse :datatype-) "maneuver/RotateToResponse")
(setf (get maneuver::RotateToResponse :definition-)
      "float32 roll
float32 pitch
float32 yaw
float32 duration
---
bool status

")



(provide :maneuver/RotateTo "e91f34039ff130f224ba3462d670a1d0")


