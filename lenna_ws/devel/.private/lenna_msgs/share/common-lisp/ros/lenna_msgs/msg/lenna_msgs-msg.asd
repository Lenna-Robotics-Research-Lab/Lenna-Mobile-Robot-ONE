
(cl:in-package :asdf)

(defsystem "lenna_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "OdomInformation" :depends-on ("_package_OdomInformation"))
    (:file "_package_OdomInformation" :depends-on ("_package"))
  ))