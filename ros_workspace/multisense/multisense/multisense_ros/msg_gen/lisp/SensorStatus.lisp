; Auto-generated. Do not edit!


(cl:in-package multisense_ros-msg)


;//! \htmlinclude SensorStatus.msg.html

(cl:defclass <SensorStatus> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (last_state_change
    :reader last_state_change
    :initarg :last_state_change
    :type cl:real
    :initform 0))
)

(cl:defclass SensorStatus (<SensorStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multisense_ros-msg:<SensorStatus> is deprecated: use multisense_ros-msg:SensorStatus instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:state-val is deprecated.  Use multisense_ros-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'last_state_change-val :lambda-list '(m))
(cl:defmethod last_state_change-val ((m <SensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:last_state_change-val is deprecated.  Use multisense_ros-msg:last_state_change instead.")
  (last_state_change m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SensorStatus>)))
    "Constants for message type '<SensorStatus>"
  '((:STOPPED . 0)
    (:RUNNING . 1)
    (:STARTING . 2)
    (:STOPPING . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SensorStatus)))
    "Constants for message type 'SensorStatus"
  '((:STOPPED . 0)
    (:RUNNING . 1)
    (:STARTING . 2)
    (:STOPPING . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorStatus>) ostream)
  "Serializes a message object of type '<SensorStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'last_state_change)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'last_state_change) (cl:floor (cl:slot-value msg 'last_state_change)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorStatus>) istream)
  "Deserializes a message object of type '<SensorStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'last_state_change) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorStatus>)))
  "Returns string type for a message object of type '<SensorStatus>"
  "multisense_ros/SensorStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorStatus)))
  "Returns string type for a message object of type 'SensorStatus"
  "multisense_ros/SensorStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorStatus>)))
  "Returns md5sum for a message object of type '<SensorStatus>"
  "0a4701d6811d800b5926beac29687bdf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorStatus)))
  "Returns md5sum for a message object of type 'SensorStatus"
  "0a4701d6811d800b5926beac29687bdf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorStatus>)))
  "Returns full string definition for message of type '<SensorStatus>"
  (cl:format cl:nil "uint8 STOPPED = 0~%uint8 RUNNING = 1~%uint8 STARTING = 2~%uint8 STOPPING = 3~%~%uint8 state~%time last_state_change~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorStatus)))
  "Returns full string definition for message of type 'SensorStatus"
  (cl:format cl:nil "uint8 STOPPED = 0~%uint8 RUNNING = 1~%uint8 STARTING = 2~%uint8 STOPPING = 3~%~%uint8 state~%time last_state_change~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorStatus>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorStatus
    (cl:cons ':state (state msg))
    (cl:cons ':last_state_change (last_state_change msg))
))
