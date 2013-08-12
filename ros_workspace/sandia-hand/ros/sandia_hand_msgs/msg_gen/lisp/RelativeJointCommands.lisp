; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RelativeJointCommands.msg.html

(cl:defclass <RelativeJointCommands> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (max_effort
    :reader max_effort
    :initarg :max_effort
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 12 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RelativeJointCommands (<RelativeJointCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RelativeJointCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RelativeJointCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RelativeJointCommands> is deprecated: use sandia_hand_msgs-msg:RelativeJointCommands instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RelativeJointCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:header-val is deprecated.  Use sandia_hand_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <RelativeJointCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:position-val is deprecated.  Use sandia_hand_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'max_effort-val :lambda-list '(m))
(cl:defmethod max_effort-val ((m <RelativeJointCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:max_effort-val is deprecated.  Use sandia_hand_msgs-msg:max_effort instead.")
  (max_effort m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RelativeJointCommands>) ostream)
  "Serializes a message object of type '<RelativeJointCommands>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'position))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'max_effort))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RelativeJointCommands>) istream)
  "Deserializes a message object of type '<RelativeJointCommands>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'position) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'position)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'max_effort) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'max_effort)))
    (cl:dotimes (i 12)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RelativeJointCommands>)))
  "Returns string type for a message object of type '<RelativeJointCommands>"
  "sandia_hand_msgs/RelativeJointCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RelativeJointCommands)))
  "Returns string type for a message object of type 'RelativeJointCommands"
  "sandia_hand_msgs/RelativeJointCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RelativeJointCommands>)))
  "Returns md5sum for a message object of type '<RelativeJointCommands>"
  "201b9e7d5b377fa3540db3fe1fc0002f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RelativeJointCommands)))
  "Returns md5sum for a message object of type 'RelativeJointCommands"
  "201b9e7d5b377fa3540db3fe1fc0002f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RelativeJointCommands>)))
  "Returns full string definition for message of type '<RelativeJointCommands>"
  (cl:format cl:nil "Header header~%float32[12] position~%uint8[12] max_effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RelativeJointCommands)))
  "Returns full string definition for message of type 'RelativeJointCommands"
  (cl:format cl:nil "Header header~%float32[12] position~%uint8[12] max_effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RelativeJointCommands>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'max_effort) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RelativeJointCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'RelativeJointCommands
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':max_effort (max_effort msg))
))
