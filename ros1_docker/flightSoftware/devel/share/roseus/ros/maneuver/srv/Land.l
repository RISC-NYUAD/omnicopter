;; Auto-generated. Do not edit!


(when (boundp 'maneuver::Land)
  (if (not (find-package "MANEUVER"))
    (make-package "MANEUVER"))
  (shadow 'Land (find-package "MANEUVER")))
(unless (find-package "MANEUVER::LAND")
  (make-package "MANEUVER::LAND"))
(unless (find-package "MANEUVER::LANDREQUEST")
  (make-package "MANEUVER::LANDREQUEST"))
(unless (find-package "MANEUVER::LANDRESPONSE")
  (make-package "MANEUVER::LANDRESPONSE"))

(in-package "ROS")





(defclass maneuver::LandRequest
  :super ros::object
  :slots (_height_1 _duration_1 _height_2 _duration_2 ))

(defmethod maneuver::LandRequest
  (:init
   (&key
    ((:height_1 __height_1) 0.0)
    ((:duration_1 __duration_1) 0.0)
    ((:height_2 __height_2) 0.0)
    ((:duration_2 __duration_2) 0.0)
    )
   (send-super :init)
   (setq _height_1 (float __height_1))
   (setq _duration_1 (float __duration_1))
   (setq _height_2 (float __height_2))
   (setq _duration_2 (float __duration_2))
   self)
  (:height_1
   (&optional __height_1)
   (if __height_1 (setq _height_1 __height_1)) _height_1)
  (:duration_1
   (&optional __duration_1)
   (if __duration_1 (setq _duration_1 __duration_1)) _duration_1)
  (:height_2
   (&optional __height_2)
   (if __height_2 (setq _height_2 __height_2)) _height_2)
  (:duration_2
   (&optional __duration_2)
   (if __duration_2 (setq _duration_2 __duration_2)) _duration_2)
  (:serialization-length
   ()
   (+
    ;; float32 _height_1
    4
    ;; float32 _duration_1
    4
    ;; float32 _height_2
    4
    ;; float32 _duration_2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _height_1
       (sys::poke _height_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration_1
       (sys::poke _duration_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height_2
       (sys::poke _height_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration_2
       (sys::poke _duration_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _height_1
     (setq _height_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration_1
     (setq _duration_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height_2
     (setq _height_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration_2
     (setq _duration_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass maneuver::LandResponse
  :super ros::object
  :slots (_status ))

(defmethod maneuver::LandResponse
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

(defclass maneuver::Land
  :super ros::object
  :slots ())

(setf (get maneuver::Land :md5sum-) "6caadcbcd5d54204c00f1a33593265c7")
(setf (get maneuver::Land :datatype-) "maneuver/Land")
(setf (get maneuver::Land :request) maneuver::LandRequest)
(setf (get maneuver::Land :response) maneuver::LandResponse)

(defmethod maneuver::LandRequest
  (:response () (instance maneuver::LandResponse :init)))

(setf (get maneuver::LandRequest :md5sum-) "6caadcbcd5d54204c00f1a33593265c7")
(setf (get maneuver::LandRequest :datatype-) "maneuver/LandRequest")
(setf (get maneuver::LandRequest :definition-)
      "float32 height_1
float32 duration_1
float32 height_2
float32 duration_2
---
bool status
")

(setf (get maneuver::LandResponse :md5sum-) "6caadcbcd5d54204c00f1a33593265c7")
(setf (get maneuver::LandResponse :datatype-) "maneuver/LandResponse")
(setf (get maneuver::LandResponse :definition-)
      "float32 height_1
float32 duration_1
float32 height_2
float32 duration_2
---
bool status
")



(provide :maneuver/Land "6caadcbcd5d54204c00f1a33593265c7")


