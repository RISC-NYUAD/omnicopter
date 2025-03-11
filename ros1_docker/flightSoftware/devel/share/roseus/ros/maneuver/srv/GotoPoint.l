;; Auto-generated. Do not edit!


(when (boundp 'maneuver::GotoPoint)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'GotoPoint (find-package "MANEUVER")))
(unless (find-package "MANEUVER::GOTOPOINT")
  (make-package "MANEUVER::GOTOPOINT"))
(unless (find-package "MANEUVER::GOTOPOINTREQUEST")
  (make-package "MANEUVER::GOTOPOINTREQUEST"))
(unless (find-package "MANEUVER::GOTOPOINTRESPONSE")
  (make-package "MANEUVER::GOTOPOINTRESPONSE"))

(in-package "ROS")





(defclass maneuver::GotoPointRequest
  :super ros::object
  :slots (_x _y _z _yaw _duration ))

(defmethod maneuver::GotoPointRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:yaw __yaw) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
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
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass maneuver::GotoPointResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::GotoPointResponse
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

(defclass maneuver::GotoPoint
  :super ros::object
  :slots ())

(setf (get maneuver::GotoPoint :md5sum-) "13a22628e95c1b6dce035f4797b346a3")
(setf (get maneuver::GotoPoint :datatype-) "maneuver/GotoPoint")
(setf (get maneuver::GotoPoint :request) maneuver::GotoPointRequest)
(setf (get maneuver::GotoPoint :response) maneuver::GotoPointResponse)

(defmethod maneuver::GotoPointRequest
  (:response () (instance maneuver::GotoPointResponse :init)))

(setf (get maneuver::GotoPointRequest :md5sum-) "13a22628e95c1b6dce035f4797b346a3")
(setf (get maneuver::GotoPointRequest :datatype-) "maneuver/GotoPointRequest")
(setf (get maneuver::GotoPointRequest :definition-)
      "float32 x
float32 y
float32 z
float32 yaw
float32 duration
---
bool status

")

(setf (get maneuver::GotoPointResponse :md5sum-) "13a22628e95c1b6dce035f4797b346a3")
(setf (get maneuver::GotoPointResponse :datatype-) "maneuver/GotoPointResponse")
(setf (get maneuver::GotoPointResponse :definition-)
      "float32 x
float32 y
float32 z
float32 yaw
float32 duration
---
bool status

")



(provide :maneuver/GotoPoint "13a22628e95c1b6dce035f4797b346a3")


