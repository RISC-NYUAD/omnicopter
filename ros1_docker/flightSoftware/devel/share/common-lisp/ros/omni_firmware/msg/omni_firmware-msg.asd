
(cl:in-package :asdf)

(defsystem "omni_firmware-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FullPose" :depends-on ("_package_FullPose"))
    (:file "_package_FullPose" :depends-on ("_package"))
    (:file "MotorSpeed" :depends-on ("_package_MotorSpeed"))
    (:file "_package_MotorSpeed" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Uvector" :depends-on ("_package_Uvector"))
    (:file "_package_Uvector" :depends-on ("_package"))
    (:file "errorMsg" :depends-on ("_package_errorMsg"))
    (:file "_package_errorMsg" :depends-on ("_package"))
  ))