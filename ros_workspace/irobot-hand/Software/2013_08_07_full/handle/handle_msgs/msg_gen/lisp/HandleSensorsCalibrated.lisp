; Auto-generated. Do not edit!


(cl:in-package handle_msgs-msg)


;//! \htmlinclude HandleSensorsCalibrated.msg.html

(cl:defclass <HandleSensorsCalibrated> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fingerTactile
    :reader fingerTactile
    :initarg :fingerTactile
    :type (cl:vector handle_msgs-msg:Finger)
   :initform (cl:make-array 3 :element-type 'handle_msgs-msg:Finger :initial-element (cl:make-instance 'handle_msgs-msg:Finger)))
   (palmTactile
    :reader palmTactile
    :initarg :palmTactile
    :type (cl:vector cl:float)
   :initform (cl:make-array 48 :element-type 'cl:float :initial-element 0.0))
   (fingerSpread
    :reader fingerSpread
    :initarg :fingerSpread
    :type cl:float
    :initform 0.0)
   (proximalJointAngle
    :reader proximalJointAngle
    :initarg :proximalJointAngle
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (distalJointAngle
    :reader distalJointAngle
    :initarg :distalJointAngle
    :type (cl:vector handle_msgs-msg:Finger)
   :initform (cl:make-array 3 :element-type 'handle_msgs-msg:Finger :initial-element (cl:make-instance 'handle_msgs-msg:Finger))))
)

(cl:defclass HandleSensorsCalibrated (<HandleSensorsCalibrated>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandleSensorsCalibrated>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandleSensorsCalibrated)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handle_msgs-msg:<HandleSensorsCalibrated> is deprecated: use handle_msgs-msg:HandleSensorsCalibrated instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HandleSensorsCalibrated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:header-val is deprecated.  Use handle_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fingerTactile-val :lambda-list '(m))
(cl:defmethod fingerTactile-val ((m <HandleSensorsCalibrated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:fingerTactile-val is deprecated.  Use handle_msgs-msg:fingerTactile instead.")
  (fingerTactile m))

(cl:ensure-generic-function 'palmTactile-val :lambda-list '(m))
(cl:defmethod palmTactile-val ((m <HandleSensorsCalibrated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:palmTactile-val is deprecated.  Use handle_msgs-msg:palmTactile instead.")
  (palmTactile m))

(cl:ensure-generic-function 'fingerSpread-val :lambda-list '(m))
(cl:defmethod fingerSpread-val ((m <HandleSensorsCalibrated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:fingerSpread-val is deprecated.  Use handle_msgs-msg:fingerSpread instead.")
  (fingerSpread m))

(cl:ensure-generic-function 'proximalJointAngle-val :lambda-list '(m))
(cl:defmethod proximalJointAngle-val ((m <HandleSensorsCalibrated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:proximalJointAngle-val is deprecated.  Use handle_msgs-msg:proximalJointAngle instead.")
  (proximalJointAngle m))

(cl:ensure-generic-function 'distalJointAngle-val :lambda-list '(m))
(cl:defmethod distalJointAngle-val ((m <HandleSensorsCalibrated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:distalJointAngle-val is deprecated.  Use handle_msgs-msg:distalJointAngle instead.")
  (distalJointAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandleSensorsCalibrated>) ostream)
  "Serializes a message object of type '<HandleSensorsCalibrated>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'fingerTactile))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'palmTactile))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fingerSpread))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'proximalJointAngle))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'distalJointAngle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandleSensorsCalibrated>) istream)
  "Deserializes a message object of type '<HandleSensorsCalibrated>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'fingerTactile) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'fingerTactile)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'handle_msgs-msg:Finger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'palmTactile) (cl:make-array 48))
  (cl:let ((vals (cl:slot-value msg 'palmTactile)))
    (cl:dotimes (i 48)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fingerSpread) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'proximalJointAngle) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'proximalJointAngle)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'distalJointAngle) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'distalJointAngle)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'handle_msgs-msg:Finger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandleSensorsCalibrated>)))
  "Returns string type for a message object of type '<HandleSensorsCalibrated>"
  "handle_msgs/HandleSensorsCalibrated")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleSensorsCalibrated)))
  "Returns string type for a message object of type 'HandleSensorsCalibrated"
  "handle_msgs/HandleSensorsCalibrated")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandleSensorsCalibrated>)))
  "Returns md5sum for a message object of type '<HandleSensorsCalibrated>"
  "54133cdcf259bf3a4416674ef7b62161")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandleSensorsCalibrated)))
  "Returns md5sum for a message object of type 'HandleSensorsCalibrated"
  "54133cdcf259bf3a4416674ef7b62161")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandleSensorsCalibrated>)))
  "Returns full string definition for message of type '<HandleSensorsCalibrated>"
  (cl:format cl:nil "# This is sensors of the HANDLE hand after calibration and data manipulation~%# published from the package sensors, by the sensors_publisher~%~%# not all the sensors were included, but only the one which were addressed at the moment~%~%# Currently only used for time stamp.  ~%Header header~%~%# The tactile array for each finger.  In units of kPa.~%# [F1, F2, F3]~%# Note there are 12 proximal and 10 distal sensors.~%Finger[3] fingerTactile~%~%# The tactile array for the palm.  In units of kPa.~%float32[48] palmTactile~%~%# The encoder on the F1 / F2 rotation.~%# Approx. 768 ticks to rotate the fingers 90 degrees.~%float32 fingerSpread~%~%# The proximal joint angle. Angle in radians~%# [F1, F2, F3]~%float32[3] proximalJointAngle~%~%# The finger distal joint flexture angle~%# [F1, F2, F3]~%# Note there are 4 readings on either side of the joint.~%Finger[3] distalJointAngle~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: handle_msgs/Finger~%# This finger definition is used for different sensors in the HandleSensors ~%# message type.  ~%~%float32[] proximal~%float32[] distal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandleSensorsCalibrated)))
  "Returns full string definition for message of type 'HandleSensorsCalibrated"
  (cl:format cl:nil "# This is sensors of the HANDLE hand after calibration and data manipulation~%# published from the package sensors, by the sensors_publisher~%~%# not all the sensors were included, but only the one which were addressed at the moment~%~%# Currently only used for time stamp.  ~%Header header~%~%# The tactile array for each finger.  In units of kPa.~%# [F1, F2, F3]~%# Note there are 12 proximal and 10 distal sensors.~%Finger[3] fingerTactile~%~%# The tactile array for the palm.  In units of kPa.~%float32[48] palmTactile~%~%# The encoder on the F1 / F2 rotation.~%# Approx. 768 ticks to rotate the fingers 90 degrees.~%float32 fingerSpread~%~%# The proximal joint angle. Angle in radians~%# [F1, F2, F3]~%float32[3] proximalJointAngle~%~%# The finger distal joint flexture angle~%# [F1, F2, F3]~%# Note there are 4 readings on either side of the joint.~%Finger[3] distalJointAngle~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: handle_msgs/Finger~%# This finger definition is used for different sensors in the HandleSensors ~%# message type.  ~%~%float32[] proximal~%float32[] distal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandleSensorsCalibrated>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fingerTactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palmTactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'proximalJointAngle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'distalJointAngle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandleSensorsCalibrated>))
  "Converts a ROS message object to a list"
  (cl:list 'HandleSensorsCalibrated
    (cl:cons ':header (header msg))
    (cl:cons ':fingerTactile (fingerTactile msg))
    (cl:cons ':palmTactile (palmTactile msg))
    (cl:cons ':fingerSpread (fingerSpread msg))
    (cl:cons ':proximalJointAngle (proximalJointAngle msg))
    (cl:cons ':distalJointAngle (distalJointAngle msg))
))
