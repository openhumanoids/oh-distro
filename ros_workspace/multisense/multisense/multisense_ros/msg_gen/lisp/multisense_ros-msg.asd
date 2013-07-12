
(cl:in-package :asdf)

(defsystem "multisense_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "RawCamData" :depends-on ("_package_RawCamData"))
    (:file "_package_RawCamData" :depends-on ("_package"))
    (:file "SensorStatus" :depends-on ("_package_SensorStatus"))
    (:file "_package_SensorStatus" :depends-on ("_package"))
    (:file "RawLidarData" :depends-on ("_package_RawLidarData"))
    (:file "_package_RawLidarData" :depends-on ("_package"))
    (:file "SensorDiagnostics" :depends-on ("_package_SensorDiagnostics"))
    (:file "_package_SensorDiagnostics" :depends-on ("_package"))
    (:file "RawCamConfig" :depends-on ("_package_RawCamConfig"))
    (:file "_package_RawCamConfig" :depends-on ("_package"))
    (:file "JointDiagnostics" :depends-on ("_package_JointDiagnostics"))
    (:file "_package_JointDiagnostics" :depends-on ("_package"))
  ))