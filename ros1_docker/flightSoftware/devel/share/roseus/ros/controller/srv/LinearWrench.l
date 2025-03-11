;; Auto-generated. Do not edit!


(when (boundp 'controller::LinearWrench)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'LinearWrench (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::LINEARWRENCH")
  (make-package "CONTROLLER::LINEARWRENCH"))
(unless (find-package "CONTROLLER::LINEARWRENCHREQUEST")
  (make-package "CONTROLLER::LINEARWRENCHREQUEST"))
(unless (find-package "CONTROLLER::LINEARWRENCHRESPONSE")
  (make-package "CONTROLLER::LINEARWRENCHRESPONSE"))

(in-package "ROS")





(defclass controller::LinearWrenchRequest
  :super ros::object
  :slots (_fx1 _fy1 _fz1 _fx2 _fy2 _fz2 _ramp _duration ))

(defmethod controller::LinearWrenchRequest
  (:init
   (&key
    ((:fx1 __fx1) 0.0)
    ((:fy1 __fy1) 0.0)
    ((:fz1 __fz1) 0.0)
    ((:fx2 __fx2) 0.0)
    ((:fy2 __fy2) 0.0)
    ((:fz2 __fz2) 0.0)
    ((:ramp __ramp) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _fx1 (float __fx1))
   (setq _fy1 (float __fy1))
   (setq _fz1 (float __fz1))
   (setq _fx2 (float __fx2))
   (setq _fy2 (float __fy2))
   (setq _fz2 (float __fz2))
   (setq _ramp (float __ramp))
   (setq _duration (float __duration))
   self)
  (:fx1
   (&optional __fx1)
   (if __fx1 (setq _fx1 __fx1)) _fx1)
  (:fy1
   (&optional __fy1)
   (if __fy1 (setq _fy1 __fy1)) _fy1)
  (:fz1
   (&optional __fz1)
   (if __fz1 (setq _fz1 __fz1)) _fz1)
  (:fx2
   (&optional __fx2)
   (if __fx2 (setq _fx2 __fx2)) _fx2)
  (:fy2
   (&optional __fy2)
   (if __fy2 (setq _fy2 __fy2)) _fy2)
  (:fz2
   (&optional __fz2)
   (if __fz2 (setq _fz2 __fz2)) _fz2)
  (:ramp
   (&optional __ramp)
   (if __ramp (setq _ramp __ramp)) _ramp)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _fx1
    4
    ;; float32 _fy1
    4
    ;; float32 _fz1
    4
    ;; float32 _fx2
    4
    ;; float32 _fy2
    4
    ;; float32 _fz2
    4
    ;; float32 _ramp
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _fx1
       (sys::poke _fx1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fy1
       (sys::poke _fy1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fz1
       (sys::poke _fz1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fx2
       (sys::poke _fx2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fy2
       (sys::poke _fy2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fz2
       (sys::poke _fz2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ramp
       (sys::poke _ramp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _fx1
     (setq _fx1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fy1
     (setq _fy1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fz1
     (setq _fz1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fx2
     (setq _fx2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fy2
     (setq _fy2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fz2
     (setq _fz2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ramp
     (setq _ramp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass controller::LinearWrenchResponse
  :super ros::object
  :slots (_status ))

(defmethod controller::LinearWrenchResponse
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

(defclass controller::LinearWrench
  :super ros::object
  :slots ())

(setf (get controller::LinearWrench :md5sum-) "676b2c4049e154bfe83fc0a28e11e13d")
(setf (get controller::LinearWrench :datatype-) "controller/LinearWrench")
(setf (get controller::LinearWrench :request) controller::LinearWrenchRequest)
(setf (get controller::LinearWrench :response) controller::LinearWrenchResponse)

(defmethod controller::LinearWrenchRequest
  (:response () (instance controller::LinearWrenchResponse :init)))

(setf (get controller::LinearWrenchRequest :md5sum-) "676b2c4049e154bfe83fc0a28e11e13d")
(setf (get controller::LinearWrenchRequest :datatype-) "controller/LinearWrenchRequest")
(setf (get controller::LinearWrenchRequest :definition-)
      "float32 fx1
float32 fy1
float32 fz1
float32 fx2
float32 fy2
float32 fz2
float32 ramp
float32 duration
---
bool status

")

(setf (get controller::LinearWrenchResponse :md5sum-) "676b2c4049e154bfe83fc0a28e11e13d")
(setf (get controller::LinearWrenchResponse :datatype-) "controller/LinearWrenchResponse")
(setf (get controller::LinearWrenchResponse :definition-)
      "float32 fx1
float32 fy1
float32 fz1
float32 fx2
float32 fy2
float32 fz2
float32 ramp
float32 duration
---
bool status

")



(provide :controller/LinearWrench "676b2c4049e154bfe83fc0a28e11e13d")


