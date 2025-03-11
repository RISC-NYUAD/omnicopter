;; Auto-generated. Do not edit!


(when (boundp 'omni_firmware::errorMsg)
  (if (not (find-package "OMNI_FIRMWARE"))
    (make-package "OMNI_FIRMWARE"))
  (shadow 'errorMsg (find-package "OMNI_FIRMWARE")))
(unless (find-package "OMNI_FIRMWARE::ERRORMSG")
  (make-package "OMNI_FIRMWARE::ERRORMSG"))

(in-package "ROS")
;;//! \htmlinclude errorMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass omni_firmware::errorMsg
  :super ros::object
  :slots (_ex _ev _ea _eR _ew _Iex _Ier _Accd _Wrench _exWrench _prop _weight ))

(defmethod omni_firmware::errorMsg
  (:init
   (&key
    ((:ex __ex) (instance geometry_msgs::Vector3 :init))
    ((:ev __ev) (instance geometry_msgs::Vector3 :init))
    ((:ea __ea) (instance geometry_msgs::Vector3 :init))
    ((:eR __eR) (instance geometry_msgs::Vector3 :init))
    ((:ew __ew) (instance geometry_msgs::Vector3 :init))
    ((:Iex __Iex) (instance geometry_msgs::Vector3 :init))
    ((:Ier __Ier) (instance geometry_msgs::Vector3 :init))
    ((:Accd __Accd) (instance geometry_msgs::Vector3 :init))
    ((:Wrench __Wrench) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:exWrench __exWrench) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:prop __prop) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:weight __weight) 0.0)
    )
   (send-super :init)
   (setq _ex __ex)
   (setq _ev __ev)
   (setq _ea __ea)
   (setq _eR __eR)
   (setq _ew __ew)
   (setq _Iex __Iex)
   (setq _Ier __Ier)
   (setq _Accd __Accd)
   (setq _Wrench __Wrench)
   (setq _exWrench __exWrench)
   (setq _prop __prop)
   (setq _weight (float __weight))
   self)
  (:ex
   (&rest __ex)
   (if (keywordp (car __ex))
       (send* _ex __ex)
     (progn
       (if __ex (setq _ex (car __ex)))
       _ex)))
  (:ev
   (&rest __ev)
   (if (keywordp (car __ev))
       (send* _ev __ev)
     (progn
       (if __ev (setq _ev (car __ev)))
       _ev)))
  (:ea
   (&rest __ea)
   (if (keywordp (car __ea))
       (send* _ea __ea)
     (progn
       (if __ea (setq _ea (car __ea)))
       _ea)))
  (:eR
   (&rest __eR)
   (if (keywordp (car __eR))
       (send* _eR __eR)
     (progn
       (if __eR (setq _eR (car __eR)))
       _eR)))
  (:ew
   (&rest __ew)
   (if (keywordp (car __ew))
       (send* _ew __ew)
     (progn
       (if __ew (setq _ew (car __ew)))
       _ew)))
  (:Iex
   (&rest __Iex)
   (if (keywordp (car __Iex))
       (send* _Iex __Iex)
     (progn
       (if __Iex (setq _Iex (car __Iex)))
       _Iex)))
  (:Ier
   (&rest __Ier)
   (if (keywordp (car __Ier))
       (send* _Ier __Ier)
     (progn
       (if __Ier (setq _Ier (car __Ier)))
       _Ier)))
  (:Accd
   (&rest __Accd)
   (if (keywordp (car __Accd))
       (send* _Accd __Accd)
     (progn
       (if __Accd (setq _Accd (car __Accd)))
       _Accd)))
  (:Wrench
   (&optional __Wrench)
   (if __Wrench (setq _Wrench __Wrench)) _Wrench)
  (:exWrench
   (&optional __exWrench)
   (if __exWrench (setq _exWrench __exWrench)) _exWrench)
  (:prop
   (&optional __prop)
   (if __prop (setq _prop __prop)) _prop)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _ex
    (send _ex :serialization-length)
    ;; geometry_msgs/Vector3 _ev
    (send _ev :serialization-length)
    ;; geometry_msgs/Vector3 _ea
    (send _ea :serialization-length)
    ;; geometry_msgs/Vector3 _eR
    (send _eR :serialization-length)
    ;; geometry_msgs/Vector3 _ew
    (send _ew :serialization-length)
    ;; geometry_msgs/Vector3 _Iex
    (send _Iex :serialization-length)
    ;; geometry_msgs/Vector3 _Ier
    (send _Ier :serialization-length)
    ;; geometry_msgs/Vector3 _Accd
    (send _Accd :serialization-length)
    ;; float32[] _Wrench
    (* 4    (length _Wrench)) 4
    ;; float32[] _exWrench
    (* 4    (length _exWrench)) 4
    ;; float32[] _prop
    (* 4    (length _prop)) 4
    ;; float32 _weight
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _ex
       (send _ex :serialize s)
     ;; geometry_msgs/Vector3 _ev
       (send _ev :serialize s)
     ;; geometry_msgs/Vector3 _ea
       (send _ea :serialize s)
     ;; geometry_msgs/Vector3 _eR
       (send _eR :serialize s)
     ;; geometry_msgs/Vector3 _ew
       (send _ew :serialize s)
     ;; geometry_msgs/Vector3 _Iex
       (send _Iex :serialize s)
     ;; geometry_msgs/Vector3 _Ier
       (send _Ier :serialize s)
     ;; geometry_msgs/Vector3 _Accd
       (send _Accd :serialize s)
     ;; float32[] _Wrench
     (write-long (length _Wrench) s)
     (dotimes (i (length _Wrench))
       (sys::poke (elt _Wrench i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _exWrench
     (write-long (length _exWrench) s)
     (dotimes (i (length _exWrench))
       (sys::poke (elt _exWrench i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _prop
     (write-long (length _prop) s)
     (dotimes (i (length _prop))
       (sys::poke (elt _prop i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _ex
     (send _ex :deserialize buf ptr-) (incf ptr- (send _ex :serialization-length))
   ;; geometry_msgs/Vector3 _ev
     (send _ev :deserialize buf ptr-) (incf ptr- (send _ev :serialization-length))
   ;; geometry_msgs/Vector3 _ea
     (send _ea :deserialize buf ptr-) (incf ptr- (send _ea :serialization-length))
   ;; geometry_msgs/Vector3 _eR
     (send _eR :deserialize buf ptr-) (incf ptr- (send _eR :serialization-length))
   ;; geometry_msgs/Vector3 _ew
     (send _ew :deserialize buf ptr-) (incf ptr- (send _ew :serialization-length))
   ;; geometry_msgs/Vector3 _Iex
     (send _Iex :deserialize buf ptr-) (incf ptr- (send _Iex :serialization-length))
   ;; geometry_msgs/Vector3 _Ier
     (send _Ier :deserialize buf ptr-) (incf ptr- (send _Ier :serialization-length))
   ;; geometry_msgs/Vector3 _Accd
     (send _Accd :deserialize buf ptr-) (incf ptr- (send _Accd :serialization-length))
   ;; float32[] _Wrench
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Wrench (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _Wrench i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _exWrench
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _exWrench (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _exWrench i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _prop
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _prop (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _prop i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _weight
     (setq _weight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get omni_firmware::errorMsg :md5sum-) "aca7c5154e5df4742f8034562501f6c5")
(setf (get omni_firmware::errorMsg :datatype-) "omni_firmware/errorMsg")
(setf (get omni_firmware::errorMsg :definition-)
      "geometry_msgs/Vector3 ex
geometry_msgs/Vector3 ev
geometry_msgs/Vector3 ea
geometry_msgs/Vector3 eR
geometry_msgs/Vector3 ew
geometry_msgs/Vector3 Iex
geometry_msgs/Vector3 Ier
geometry_msgs/Vector3 Accd
float32[] Wrench
float32[] exWrench
float32[] prop
float32 weight

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :omni_firmware/errorMsg "aca7c5154e5df4742f8034562501f6c5")


