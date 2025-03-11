;; Auto-generated. Do not edit!


(when (boundp 'maneuver::LiftOff)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'LiftOff (find-package "MANEUVER")))
(unless (find-package "MANEUVER::LIFTOFF")
  (make-package "MANEUVER::LIFTOFF"))
(unless (find-package "MANEUVER::LIFTOFFREQUEST")
  (make-package "MANEUVER::LIFTOFFREQUEST"))
(unless (find-package "MANEUVER::LIFTOFFRESPONSE")
  (make-package "MANEUVER::LIFTOFFRESPONSE"))

(in-package "ROS")





(defclass maneuver::LiftOffRequest
  :super ros::object
  :slots (_height _duration ))

(defmethod maneuver::LiftOffRequest
  (:init
   (&key
    ((:height __height) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _height (float __height))
   (setq _duration (float __duration))
   self)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _height
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass maneuver::LiftOffResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::LiftOffResponse
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

(defclass maneuver::LiftOff
  :super ros::object
  :slots ())

(setf (get maneuver::LiftOff :md5sum-) "9e16cfae3e3728d84a285e6768841653")
(setf (get maneuver::LiftOff :datatype-) "maneuver/LiftOff")
(setf (get maneuver::LiftOff :request) maneuver::LiftOffRequest)
(setf (get maneuver::LiftOff :response) maneuver::LiftOffResponse)

(defmethod maneuver::LiftOffRequest
  (:response () (instance maneuver::LiftOffResponse :init)))

(setf (get maneuver::LiftOffRequest :md5sum-) "9e16cfae3e3728d84a285e6768841653")
(setf (get maneuver::LiftOffRequest :datatype-) "maneuver/LiftOffRequest")
(setf (get maneuver::LiftOffRequest :definition-)
      "float32 height
float32 duration
---
bool status

")

(setf (get maneuver::LiftOffResponse :md5sum-) "9e16cfae3e3728d84a285e6768841653")
(setf (get maneuver::LiftOffResponse :datatype-) "maneuver/LiftOffResponse")
(setf (get maneuver::LiftOffResponse :definition-)
      "float32 height
float32 duration
---
bool status

")



(provide :maneuver/LiftOff "9e16cfae3e3728d84a285e6768841653")


