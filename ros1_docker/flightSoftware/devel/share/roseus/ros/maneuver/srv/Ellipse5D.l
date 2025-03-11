;; Auto-generated. Do not edit!


(when (boundp 'maneuver::Ellipse5D)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'Ellipse5D (find-package "MANEUVER")))
(unless (find-package "MANEUVER::ELLIPSE5D")
  (make-package "MANEUVER::ELLIPSE5D"))
(unless (find-package "MANEUVER::ELLIPSE5DREQUEST")
  (make-package "MANEUVER::ELLIPSE5DREQUEST"))
(unless (find-package "MANEUVER::ELLIPSE5DRESPONSE")
  (make-package "MANEUVER::ELLIPSE5DRESPONSE"))

(in-package "ROS")





(defclass maneuver::Ellipse5DRequest
  :super ros::object
  :slots (_x_min _x_max _y_min _y_max _z_min _z_max _roll_start _roll_mid _roll_end _pitch_start _pitch_mid _pitch_end _yaw _duration ))

(defmethod maneuver::Ellipse5DRequest
  (:init
   (&key
    ((:x_min __x_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_min __y_min) 0.0)
    ((:y_max __y_max) 0.0)
    ((:z_min __z_min) 0.0)
    ((:z_max __z_max) 0.0)
    ((:roll_start __roll_start) 0.0)
    ((:roll_mid __roll_mid) 0.0)
    ((:roll_end __roll_end) 0.0)
    ((:pitch_start __pitch_start) 0.0)
    ((:pitch_mid __pitch_mid) 0.0)
    ((:pitch_end __pitch_end) 0.0)
    ((:yaw __yaw) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _x_min (float __x_min))
   (setq _x_max (float __x_max))
   (setq _y_min (float __y_min))
   (setq _y_max (float __y_max))
   (setq _z_min (float __z_min))
   (setq _z_max (float __z_max))
   (setq _roll_start (float __roll_start))
   (setq _roll_mid (float __roll_mid))
   (setq _roll_end (float __roll_end))
   (setq _pitch_start (float __pitch_start))
   (setq _pitch_mid (float __pitch_mid))
   (setq _pitch_end (float __pitch_end))
   (setq _yaw (float __yaw))
   (setq _duration (float __duration))
   self)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:z_min
   (&optional __z_min)
   (if __z_min (setq _z_min __z_min)) _z_min)
  (:z_max
   (&optional __z_max)
   (if __z_max (setq _z_max __z_max)) _z_max)
  (:roll_start
   (&optional __roll_start)
   (if __roll_start (setq _roll_start __roll_start)) _roll_start)
  (:roll_mid
   (&optional __roll_mid)
   (if __roll_mid (setq _roll_mid __roll_mid)) _roll_mid)
  (:roll_end
   (&optional __roll_end)
   (if __roll_end (setq _roll_end __roll_end)) _roll_end)
  (:pitch_start
   (&optional __pitch_start)
   (if __pitch_start (setq _pitch_start __pitch_start)) _pitch_start)
  (:pitch_mid
   (&optional __pitch_mid)
   (if __pitch_mid (setq _pitch_mid __pitch_mid)) _pitch_mid)
  (:pitch_end
   (&optional __pitch_end)
   (if __pitch_end (setq _pitch_end __pitch_end)) _pitch_end)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _x_min
    4
    ;; float32 _x_max
    4
    ;; float32 _y_min
    4
    ;; float32 _y_max
    4
    ;; float32 _z_min
    4
    ;; float32 _z_max
    4
    ;; float32 _roll_start
    4
    ;; float32 _roll_mid
    4
    ;; float32 _roll_end
    4
    ;; float32 _pitch_start
    4
    ;; float32 _pitch_mid
    4
    ;; float32 _pitch_end
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
     ;; float32 _x_min
       (sys::poke _x_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_max
       (sys::poke _x_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_min
       (sys::poke _y_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_max
       (sys::poke _y_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_min
       (sys::poke _z_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_max
       (sys::poke _z_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_start
       (sys::poke _roll_start (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_mid
       (sys::poke _roll_mid (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_end
       (sys::poke _roll_end (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_start
       (sys::poke _pitch_start (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_mid
       (sys::poke _pitch_mid (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_end
       (sys::poke _pitch_end (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x_min
     (setq _x_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_max
     (setq _x_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_min
     (setq _y_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_max
     (setq _y_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_min
     (setq _z_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_max
     (setq _z_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_start
     (setq _roll_start (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_mid
     (setq _roll_mid (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_end
     (setq _roll_end (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_start
     (setq _pitch_start (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_mid
     (setq _pitch_mid (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_end
     (setq _pitch_end (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass maneuver::Ellipse5DResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::Ellipse5DResponse
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

(defclass maneuver::Ellipse5D
  :super ros::object
  :slots ())

(setf (get maneuver::Ellipse5D :md5sum-) "bdb0ce274c1da7c8239ea3857b021bbb")
(setf (get maneuver::Ellipse5D :datatype-) "maneuver/Ellipse5D")
(setf (get maneuver::Ellipse5D :request) maneuver::Ellipse5DRequest)
(setf (get maneuver::Ellipse5D :response) maneuver::Ellipse5DResponse)

(defmethod maneuver::Ellipse5DRequest
  (:response () (instance maneuver::Ellipse5DResponse :init)))

(setf (get maneuver::Ellipse5DRequest :md5sum-) "bdb0ce274c1da7c8239ea3857b021bbb")
(setf (get maneuver::Ellipse5DRequest :datatype-) "maneuver/Ellipse5DRequest")
(setf (get maneuver::Ellipse5DRequest :definition-)
      "float32 x_min
float32 x_max
float32 y_min
float32 y_max
float32 z_min
float32 z_max
float32 roll_start
float32 roll_mid
float32 roll_end
float32 pitch_start
float32 pitch_mid
float32 pitch_end
float32 yaw
float32 duration
---
bool status

")

(setf (get maneuver::Ellipse5DResponse :md5sum-) "bdb0ce274c1da7c8239ea3857b021bbb")
(setf (get maneuver::Ellipse5DResponse :datatype-) "maneuver/Ellipse5DResponse")
(setf (get maneuver::Ellipse5DResponse :definition-)
      "float32 x_min
float32 x_max
float32 y_min
float32 y_max
float32 z_min
float32 z_max
float32 roll_start
float32 roll_mid
float32 roll_end
float32 pitch_start
float32 pitch_mid
float32 pitch_end
float32 yaw
float32 duration
---
bool status

")



(provide :maneuver/Ellipse5D "bdb0ce274c1da7c8239ea3857b021bbb")


