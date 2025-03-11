;; Auto-generated. Do not edit!


(when (boundp 'controller::AngularWrench)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'AngularWrench (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::ANGULARWRENCH")
  (make-package "CONTROLLER::ANGULARWRENCH"))
(unless (find-package "CONTROLLER::ANGULARWRENCHREQUEST")
  (make-package "CONTROLLER::ANGULARWRENCHREQUEST"))
(unless (find-package "CONTROLLER::ANGULARWRENCHRESPONSE")
  (make-package "CONTROLLER::ANGULARWRENCHRESPONSE"))

(in-package "ROS")





(defclass controller::AngularWrenchRequest
  :super ros::object
  :slots (_fz _phi1 _phi2 _duration ))

(defmethod controller::AngularWrenchRequest
  (:init
   (&key
    ((:fz __fz) 0.0)
    ((:phi1 __phi1) 0.0)
    ((:phi2 __phi2) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _fz (float __fz))
   (setq _phi1 (float __phi1))
   (setq _phi2 (float __phi2))
   (setq _duration (float __duration))
   self)
  (:fz
   (&optional __fz)
   (if __fz (setq _fz __fz)) _fz)
  (:phi1
   (&optional __phi1)
   (if __phi1 (setq _phi1 __phi1)) _phi1)
  (:phi2
   (&optional __phi2)
   (if __phi2 (setq _phi2 __phi2)) _phi2)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _fz
    4
    ;; float32 _phi1
    4
    ;; float32 _phi2
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _fz
       (sys::poke _fz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi1
       (sys::poke _phi1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi2
       (sys::poke _phi2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _fz
     (setq _fz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi1
     (setq _phi1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi2
     (setq _phi2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass controller::AngularWrenchResponse
  :super ros::object
  :slots (_status ))

(defmethod controller::AngularWrenchResponse
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

(defclass controller::AngularWrench
  :super ros::object
  :slots ())

(setf (get controller::AngularWrench :md5sum-) "bfbfde29636da2ef0700941fae523d7c")
(setf (get controller::AngularWrench :datatype-) "controller/AngularWrench")
(setf (get controller::AngularWrench :request) controller::AngularWrenchRequest)
(setf (get controller::AngularWrench :response) controller::AngularWrenchResponse)

(defmethod controller::AngularWrenchRequest
  (:response () (instance controller::AngularWrenchResponse :init)))

(setf (get controller::AngularWrenchRequest :md5sum-) "bfbfde29636da2ef0700941fae523d7c")
(setf (get controller::AngularWrenchRequest :datatype-) "controller/AngularWrenchRequest")
(setf (get controller::AngularWrenchRequest :definition-)
      "float32 fz
float32 phi1
float32 phi2
float32 duration
---
bool status

")

(setf (get controller::AngularWrenchResponse :md5sum-) "bfbfde29636da2ef0700941fae523d7c")
(setf (get controller::AngularWrenchResponse :datatype-) "controller/AngularWrenchResponse")
(setf (get controller::AngularWrenchResponse :definition-)
      "float32 fz
float32 phi1
float32 phi2
float32 duration
---
bool status

")



(provide :controller/AngularWrench "bfbfde29636da2ef0700941fae523d7c")


