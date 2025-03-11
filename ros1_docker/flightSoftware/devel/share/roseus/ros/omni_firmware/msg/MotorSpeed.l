;; Auto-generated. Do not edit!


(when (boundp 'omni_firmware::MotorSpeed)
  (if (not (find-package "OMNI_FIRMWARE"))
    (make-package "OMNI_FIRMWARE"))
  (shadow 'MotorSpeed (find-package "OMNI_FIRMWARE")))
(unless (find-package "OMNI_FIRMWARE::MOTORSPEED")
  (make-package "OMNI_FIRMWARE::MOTORSPEED"))

(in-package "ROS")
;;//! \htmlinclude MotorSpeed.msg.html


(defclass omni_firmware::MotorSpeed
  :super ros::object
  :slots (_name _velocity ))

(defmethod omni_firmware::MotorSpeed
  (:init
   (&key
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _name __name)
   (setq _velocity __velocity)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float32[] _velocity
    (* 4    (length _velocity)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get omni_firmware::MotorSpeed :md5sum-) "f1d54c3c8a939a204eb96629420d7ab7")
(setf (get omni_firmware::MotorSpeed :datatype-) "omni_firmware/MotorSpeed")
(setf (get omni_firmware::MotorSpeed :definition-)
      "string[] name
float32[] velocity

")



(provide :omni_firmware/MotorSpeed "f1d54c3c8a939a204eb96629420d7ab7")


