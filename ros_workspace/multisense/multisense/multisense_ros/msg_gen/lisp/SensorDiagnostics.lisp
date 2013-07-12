; Auto-generated. Do not edit!


(cl:in-package multisense_ros-msg)


;//! \htmlinclude SensorDiagnostics.msg.html

(cl:defclass <SensorDiagnostics> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type multisense_ros-msg:SensorStatus
    :initform (cl:make-instance 'multisense_ros-msg:SensorStatus))
   (desired_rate
    :reader desired_rate
    :initarg :desired_rate
    :type cl:float
    :initform 0.0)
   (current_rate
    :reader current_rate
    :initarg :current_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass SensorDiagnostics (<SensorDiagnostics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorDiagnostics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorDiagnostics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multisense_ros-msg:<SensorDiagnostics> is deprecated: use multisense_ros-msg:SensorDiagnostics instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <SensorDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:stamp-val is deprecated.  Use multisense_ros-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SensorDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:status-val is deprecated.  Use multisense_ros-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'desired_rate-val :lambda-list '(m))
(cl:defmethod desired_rate-val ((m <SensorDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:desired_rate-val is deprecated.  Use multisense_ros-msg:desired_rate instead.")
  (desired_rate m))

(cl:ensure-generic-function 'current_rate-val :lambda-list '(m))
(cl:defmethod current_rate-val ((m <SensorDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:current_rate-val is deprecated.  Use multisense_ros-msg:current_rate instead.")
  (current_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorDiagnostics>) ostream)
  "Serializes a message object of type '<SensorDiagnostics>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desired_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorDiagnostics>) istream)
  "Deserializes a message object of type '<SensorDiagnostics>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desired_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorDiagnostics>)))
  "Returns string type for a message object of type '<SensorDiagnostics>"
  "multisense_ros/SensorDiagnostics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorDiagnostics)))
  "Returns string type for a message object of type 'SensorDiagnostics"
  "multisense_ros/SensorDiagnostics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorDiagnostics>)))
  "Returns md5sum for a message object of type '<SensorDiagnostics>"
  "28e7c3ade6fe630a725df4dd4637fe9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorDiagnostics)))
  "Returns md5sum for a message object of type 'SensorDiagnostics"
  "28e7c3ade6fe630a725df4dd4637fe9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorDiagnostics>)))
  "Returns full string definition for message of type '<SensorDiagnostics>"
  (cl:format cl:nil "time stamp~%multisense_ros/SensorStatus status~%float32 desired_rate~%float32 current_rate~%~%================================================================================~%MSG: multisense_ros/SensorStatus~%uint8 STOPPED = 0~%uint8 RUNNING = 1~%uint8 STARTING = 2~%uint8 STOPPING = 3~%~%uint8 state~%time last_state_change~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorDiagnostics)))
  "Returns full string definition for message of type 'SensorDiagnostics"
  (cl:format cl:nil "time stamp~%multisense_ros/SensorStatus status~%float32 desired_rate~%float32 current_rate~%~%================================================================================~%MSG: multisense_ros/SensorStatus~%uint8 STOPPED = 0~%uint8 RUNNING = 1~%uint8 STARTING = 2~%uint8 STOPPING = 3~%~%uint8 state~%time last_state_change~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorDiagnostics>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorDiagnostics>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorDiagnostics
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':status (status msg))
    (cl:cons ':desired_rate (desired_rate msg))
    (cl:cons ':current_rate (current_rate msg))
))
