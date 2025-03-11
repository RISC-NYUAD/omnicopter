
(cl:in-package :asdf)

(defsystem "servo_control_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReleaseArms" :depends-on ("_package_ReleaseArms"))
    (:file "_package_ReleaseArms" :depends-on ("_package"))
    (:file "SetAngles" :depends-on ("_package_SetAngles"))
    (:file "_package_SetAngles" :depends-on ("_package"))
  ))