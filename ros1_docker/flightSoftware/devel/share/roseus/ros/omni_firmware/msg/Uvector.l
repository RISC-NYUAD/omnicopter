;; Auto-generated. Do not edit!


(when (boundp 'omni_firmware::Uvector)
  (if (not (find-package "OMNI_FIRMWARE"))
    (make-package "OMNI_FIRMWARE"))
  (shadow 'Uvector (find-package "OMNI_FIRMWARE")))
(unless (find-package "OMNI_FIRMWARE::UVECTOR")
  (make-package "OMNI_FIRMWARE::UVECTOR"))

(in-package "ROS")
;;//! \htmlinclude Uvector.msg.html


(defclass omni_firmware::Uvector
  :super ros::object
  :slots (_value ))

(defmethod omni_firmware::Uvector
  (:init
   (&key
    ((:value __value) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _value __value)
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; float32[12] _value
    (* 4    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[12] _value
     (dotimes (i 12)
       (sys::poke (elt _value i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[12] _value
   (dotimes (i (length _value))
     (setf (elt _value i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get omni_firmware::Uvector :md5sum-) "3cac8a89b50e7d740ace6fafac0a09de")
(setf (get omni_firmware::Uvector :datatype-) "omni_firmware/Uvector")
(setf (get omni_firmware::Uvector :definition-)
      "float32[12] value

")



(provide :omni_firmware/Uvector "3cac8a89b50e7d740ace6fafac0a09de")


