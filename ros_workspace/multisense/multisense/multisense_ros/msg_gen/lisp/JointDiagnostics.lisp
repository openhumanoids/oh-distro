; Auto-generated. Do not edit!


(cl:in-package multisense_ros-msg)


;//! \htmlinclude JointDiagnostics.msg.html

(cl:defclass <JointDiagnostics> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (desired_rate
    :reader desired_rate
    :initarg :desired_rate
    :type cl:float
    :initform 0.0)
   (current_rate
    :reader current_rate
    :initarg :current_rate
    :type cl:float
    :initform 0.0)
   (joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass JointDiagnostics (<JointDiagnostics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointDiagnostics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointDiagnostics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multisense_ros-msg:<JointDiagnostics> is deprecated: use multisense_ros-msg:JointDiagnostics instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <JointDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:stamp-val is deprecated.  Use multisense_ros-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'desired_rate-val :lambda-list '(m))
(cl:defmethod desired_rate-val ((m <JointDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:desired_rate-val is deprecated.  Use multisense_ros-msg:desired_rate instead.")
  (desired_rate m))

(cl:ensure-generic-function 'current_rate-val :lambda-list '(m))
(cl:defmethod current_rate-val ((m <JointDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:current_rate-val is deprecated.  Use multisense_ros-msg:current_rate instead.")
  (current_rate m))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <JointDiagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:joint_state-val is deprecated.  Use multisense_ros-msg:joint_state instead.")
  (joint_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointDiagnostics>) ostream)
  "Serializes a message object of type '<JointDiagnostics>"
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointDiagnostics>) istream)
  "Deserializes a message object of type '<JointDiagnostics>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointDiagnostics>)))
  "Returns string type for a message object of type '<JointDiagnostics>"
  "multisense_ros/JointDiagnostics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointDiagnostics)))
  "Returns string type for a message object of type 'JointDiagnostics"
  "multisense_ros/JointDiagnostics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointDiagnostics>)))
  "Returns md5sum for a message object of type '<JointDiagnostics>"
  "0e112096640d0aa02c36ee43b8a4feea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointDiagnostics)))
  "Returns md5sum for a message object of type 'JointDiagnostics"
  "0e112096640d0aa02c36ee43b8a4feea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointDiagnostics>)))
  "Returns full string definition for message of type '<JointDiagnostics>"
  (cl:format cl:nil "time stamp~%float32 desired_rate~%float32 current_rate~%~%sensor_msgs/JointState joint_state~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointDiagnostics)))
  "Returns full string definition for message of type 'JointDiagnostics"
  (cl:format cl:nil "time stamp~%float32 desired_rate~%float32 current_rate~%~%sensor_msgs/JointState joint_state~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointDiagnostics>))
  (cl:+ 0
     8
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointDiagnostics>))
  "Converts a ROS message object to a list"
  (cl:list 'JointDiagnostics
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':desired_rate (desired_rate msg))
    (cl:cons ':current_rate (current_rate msg))
    (cl:cons ':joint_state (joint_state msg))
))
