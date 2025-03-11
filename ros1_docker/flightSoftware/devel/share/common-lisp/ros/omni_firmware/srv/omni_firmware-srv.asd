
(cl:in-package :asdf)

(defsystem "omni_firmware-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ZeroUKF" :depends-on ("_package_ZeroUKF"))
    (:file "_package_ZeroUKF" :depends-on ("_package"))
    (:file "arm" :depends-on ("_package_arm"))
    (:file "_package_arm" :depends-on ("_package"))
    (:file "disarm" :depends-on ("_package_disarm"))
    (:file "_package_disarm" :depends-on ("_package"))
  ))