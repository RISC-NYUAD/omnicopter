;; Auto-generated. Do not edit!


(when (boundp 'maneuver::Goto6DPoint)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'Goto6DPoint (find-package "MANEUVER")))
(unless (find-package "MANEUVER::GOTO6DPOINT")
  (make-package "MANEUVER::GOTO6DPOINT"))
(unless (find-package "MANEUVER::GOTO6DPOINTREQUEST")
  (make-package "MANEUVER::GOTO6DPOINTREQUEST"))
(unless (find-package "MANEUVER::GOTO6DPOINTRESPONSE")
  (make-package "MANEUVER::GOTO6DPOINTRESPONSE"))

(in-package "ROS")





(defclass maneuver::Goto6DPointRequest
  :super ros::object
  :slots (_x _y _z _roll _pitch _yaw _duration ))

(defmethod maneuver::Goto6DPointRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _duration (float __duration))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
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
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
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
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
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

(defclass maneuver::Goto6DPointResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::Goto6DPointResponse
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

(defclass maneuver::Goto6DPoint
  :super ros::object
  :slots ())

(setf (get maneuver::Goto6DPoint :md5sum-) "ceed9bb0a85fabd7e22e37d55fef957b")
(setf (get maneuver::Goto6DPoint :datatype-) "maneuver/Goto6DPoint")
(setf (get maneuver::Goto6DPoint :request) maneuver::Goto6DPointRequest)
(setf (get maneuver::Goto6DPoint :response) maneuver::Goto6DPointResponse)

(defmethod maneuver::Goto6DPointRequest
  (:response () (instance maneuver::Goto6DPointResponse :init)))

(setf (get maneuver::Goto6DPointRequest :md5sum-) "ceed9bb0a85fabd7e22e37d55fef957b")
(setf (get maneuver::Goto6DPointRequest :datatype-) "maneuver/Goto6DPointRequest")
(setf (get maneuver::Goto6DPointRequest :definition-)
      "float32 x
float32 y
float32 z
float32 roll
float32 pitch
float32 yaw
float32 duration
---
bool status

")

(setf (get maneuver::Goto6DPointResponse :md5sum-) "ceed9bb0a85fabd7e22e37d55fef957b")
(setf (get maneuver::Goto6DPointResponse :datatype-) "maneuver/Goto6DPointResponse")
(setf (get maneuver::Goto6DPointResponse :definition-)
      "float32 x
float32 y
float32 z
float32 roll
float32 pitch
float32 yaw
float32 duration
---
bool status

")



(provide :maneuver/Goto6DPoint "ceed9bb0a85fabd7e22e37d55fef957b")


