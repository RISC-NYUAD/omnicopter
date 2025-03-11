;; Auto-generated. Do not edit!


(when (boundp 'servo_control_node::SetAngles)
  (if (not (find-package "SERVO_CONTROL_NODE"))
    (make-package "SERVO_CONTROL_NODE"))
  (shadow 'SetAngles (find-package "SERVO_CONTROL_NODE")))
(unless (find-package "SERVO_CONTROL_NODE::SETANGLES")
  (make-package "SERVO_CONTROL_NODE::SETANGLES"))
(unless (find-package "SERVO_CONTROL_NODE::SETANGLESREQUEST")
  (make-package "SERVO_CONTROL_NODE::SETANGLESREQUEST"))
(unless (find-package "SERVO_CONTROL_NODE::SETANGLESRESPONSE")
  (make-package "SERVO_CONTROL_NODE::SETANGLESRESPONSE"))

(in-package "ROS")





(defclass servo_control_node::SetAnglesRequest
  :super ros::object
  :slots (_right _left ))

(defmethod servo_control_node::SetAnglesRequest
  (:init
   (&key
    ((:right __right) 0)
    ((:left __left) 0)
    )
   (send-super :init)
   (setq _right (round __right))
   (setq _left (round __left))
   self)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:serialization-length
   ()
   (+
    ;; int32 _right
    4
    ;; int32 _left
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _right
       (write-long _right s)
     ;; int32 _left
       (write-long _left s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _right
     (setq _right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _left
     (setq _left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass servo_control_node::SetAnglesResponse
  :super ros::object
  :slots (_success ))

(defmethod servo_control_node::SetAnglesResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass servo_control_node::SetAngles
  :super ros::object
  :slots ())

(setf (get servo_control_node::SetAngles :md5sum-) "af6660be3609fe9894731472db7c883b")
(setf (get servo_control_node::SetAngles :datatype-) "servo_control_node/SetAngles")
(setf (get servo_control_node::SetAngles :request) servo_control_node::SetAnglesRequest)
(setf (get servo_control_node::SetAngles :response) servo_control_node::SetAnglesResponse)

(defmethod servo_control_node::SetAnglesRequest
  (:response () (instance servo_control_node::SetAnglesResponse :init)))

(setf (get servo_control_node::SetAnglesRequest :md5sum-) "af6660be3609fe9894731472db7c883b")
(setf (get servo_control_node::SetAnglesRequest :datatype-) "servo_control_node/SetAnglesRequest")
(setf (get servo_control_node::SetAnglesRequest :definition-)
      "# srv/SetAngles.srv

int32 right   # Angle for the right motor
int32 left    # Angle for the left motor
---
bool success  # Return success if the operation was successful


")

(setf (get servo_control_node::SetAnglesResponse :md5sum-) "af6660be3609fe9894731472db7c883b")
(setf (get servo_control_node::SetAnglesResponse :datatype-) "servo_control_node/SetAnglesResponse")
(setf (get servo_control_node::SetAnglesResponse :definition-)
      "# srv/SetAngles.srv

int32 right   # Angle for the right motor
int32 left    # Angle for the left motor
---
bool success  # Return success if the operation was successful


")



(provide :servo_control_node/SetAngles "af6660be3609fe9894731472db7c883b")


