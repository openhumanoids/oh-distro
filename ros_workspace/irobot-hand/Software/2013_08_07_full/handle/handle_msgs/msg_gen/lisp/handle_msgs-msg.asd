
(cl:in-package :asdf)

(defsystem "handle_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HandleControl" :depends-on ("_package_HandleControl"))
    (:file "_package_HandleControl" :depends-on ("_package"))
    (:file "HandleSensorsCalibrated" :depends-on ("_package_HandleSensorsCalibrated"))
    (:file "_package_HandleSensorsCalibrated" :depends-on ("_package"))
    (:file "HandleCollisions" :depends-on ("_package_HandleCollisions"))
    (:file "_package_HandleCollisions" :depends-on ("_package"))
    (:file "CableTension" :depends-on ("_package_CableTension"))
    (:file "_package_CableTension" :depends-on ("_package"))
    (:file "Collision" :depends-on ("_package_Collision"))
    (:file "_package_Collision" :depends-on ("_package"))
    (:file "HandleSensors" :depends-on ("_package_HandleSensors"))
    (:file "_package_HandleSensors" :depends-on ("_package"))
    (:file "Finger" :depends-on ("_package_Finger"))
    (:file "_package_Finger" :depends-on ("_package"))
  ))