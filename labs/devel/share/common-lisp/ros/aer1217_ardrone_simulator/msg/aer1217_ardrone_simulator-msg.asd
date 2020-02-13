
(cl:in-package :asdf)

(defsystem "aer1217_ardrone_simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DesiredStateMsg" :depends-on ("_package_DesiredStateMsg"))
    (:file "_package_DesiredStateMsg" :depends-on ("_package"))
    (:file "GazeboState" :depends-on ("_package_GazeboState"))
    (:file "_package_GazeboState" :depends-on ("_package"))
    (:file "MotorCommands" :depends-on ("_package_MotorCommands"))
    (:file "_package_MotorCommands" :depends-on ("_package"))
  ))