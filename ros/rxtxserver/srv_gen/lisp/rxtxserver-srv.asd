
(cl:in-package :asdf)

(defsystem "rxtxserver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Read" :depends-on ("_package_Read"))
    (:file "_package_Read" :depends-on ("_package"))
    (:file "Byte" :depends-on ("_package_Byte"))
    (:file "_package_Byte" :depends-on ("_package"))
  ))