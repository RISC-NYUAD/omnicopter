;; Auto-generated. Do not edit!


(when (boundp 'maneuver::FullFlip)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'FullFlip (find-package "MANEUVER")))
(unless (find-package "MANEUVER::FULLFLIP")
  (make-package "MANEUVER::FULLFLIP"))
(unless (find-package "MANEUVER::FULLFLIPREQUEST")
  (make-package "MANEUVER::FULLFLIPREQUEST"))
(unless (find-package "MANEUVER::FULLFLIPRESPONSE")
  (make-package "MANEUVER::FULLFLIPRESPONSE"))

(in-package "ROS")





(defclass maneuver::FullFlipRequest
  :super ros::object
  :slots (_roll_bool _pitch_bool _duration ))

(defmethod maneuver::FullFlipRequest
  (:init
   (&key
    ((:roll_bool __roll_bool) nil)
    ((:pitch_bool __pitch_bool) nil)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _roll_bool __roll_bool)
   (setq _pitch_bool __pitch_bool)
   (setq _duration (float __duration))
   self)
  (:roll_bool
   (&optional (__roll_bool :null))
   (if (not (eq __roll_bool :null)) (setq _roll_bool __roll_bool)) _roll_bool)
  (:pitch_bool
   (&optional (__pitch_bool :null))
   (if (not (eq __pitch_bool :null)) (setq _pitch_bool __pitch_bool)) _pitch_bool)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; bool _roll_bool
    1
    ;; bool _pitch_bool
    1
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _roll_bool
       (if _roll_bool (write-byte -1 s) (write-byte 0 s))
     ;; bool _pitch_bool
       (if _pitch_bool (write-byte -1 s) (write-byte 0 s))
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _roll_bool
     (setq _roll_bool (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _pitch_bool
     (setq _pitch_bool (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass maneuver::FullFlipResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::FullFlipResponse
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

(defclass maneuver::FullFlip
  :super ros::object
  :slots ())

(setf (get maneuver::FullFlip :md5sum-) "432212f34386b2b3d536658964c298f8")
(setf (get maneuver::FullFlip :datatype-) "maneuver/FullFlip")
(setf (get maneuver::FullFlip :request) maneuver::FullFlipRequest)
(setf (get maneuver::FullFlip :response) maneuver::FullFlipResponse)

(defmethod maneuver::FullFlipRequest
  (:response () (instance maneuver::FullFlipResponse :init)))

(setf (get maneuver::FullFlipRequest :md5sum-) "432212f34386b2b3d536658964c298f8")
(setf (get maneuver::FullFlipRequest :datatype-) "maneuver/FullFlipRequest")
(setf (get maneuver::FullFlipRequest :definition-)
      "bool roll_bool
bool pitch_bool
float32 duration
---
bool status

")

(setf (get maneuver::FullFlipResponse :md5sum-) "432212f34386b2b3d536658964c298f8")
(setf (get maneuver::FullFlipResponse :datatype-) "maneuver/FullFlipResponse")
(setf (get maneuver::FullFlipResponse :definition-)
      "bool roll_bool
bool pitch_bool
float32 duration
---
bool status

")



(provide :maneuver/FullFlip "432212f34386b2b3d536658964c298f8")


