
(cl:in-package :asdf)

(defsystem "controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AngularWrench" :depends-on ("_package_AngularWrench"))
    (:file "_package_AngularWrench" :depends-on ("_package"))
    (:file "LinearWrench" :depends-on ("_package_LinearWrench"))
    (:file "_package_LinearWrench" :depends-on ("_package"))
  ))