
(cl:in-package :asdf)

(defsystem "rxtxserver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AngularPos" :depends-on ("_package_AngularPos"))
    (:file "_package_AngularPos" :depends-on ("_package"))
    (:file "status" :depends-on ("_package_status"))
    (:file "_package_status" :depends-on ("_package"))
    (:file "distance" :depends-on ("_package_distance"))
    (:file "_package_distance" :depends-on ("_package"))
  ))