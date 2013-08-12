; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RawTactile.msg.html

(cl:defclass <RawTactile> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (f0
    :reader f0
    :initarg :f0
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (f1
    :reader f1
    :initarg :f1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (f2
    :reader f2
    :initarg :f2
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (f3
    :reader f3
    :initarg :f3
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (palm
    :reader palm
    :initarg :palm
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawTactile (<RawTactile>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawTactile>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawTactile)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RawTactile> is deprecated: use sandia_hand_msgs-msg:RawTactile instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RawTactile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:header-val is deprecated.  Use sandia_hand_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'f0-val :lambda-list '(m))
(cl:defmethod f0-val ((m <RawTactile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:f0-val is deprecated.  Use sandia_hand_msgs-msg:f0 instead.")
  (f0 m))

(cl:ensure-generic-function 'f1-val :lambda-list '(m))
(cl:defmethod f1-val ((m <RawTactile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:f1-val is deprecated.  Use sandia_hand_msgs-msg:f1 instead.")
  (f1 m))

(cl:ensure-generic-function 'f2-val :lambda-list '(m))
(cl:defmethod f2-val ((m <RawTactile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:f2-val is deprecated.  Use sandia_hand_msgs-msg:f2 instead.")
  (f2 m))

(cl:ensure-generic-function 'f3-val :lambda-list '(m))
(cl:defmethod f3-val ((m <RawTactile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:f3-val is deprecated.  Use sandia_hand_msgs-msg:f3 instead.")
  (f3 m))

(cl:ensure-generic-function 'palm-val :lambda-list '(m))
(cl:defmethod palm-val ((m <RawTactile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm-val is deprecated.  Use sandia_hand_msgs-msg:palm instead.")
  (palm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawTactile>) ostream)
  "Serializes a message object of type '<RawTactile>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'f0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'f0))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'f1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'f1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'f2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'f2))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'f3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'f3))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'palm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'palm))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawTactile>) istream)
  "Deserializes a message object of type '<RawTactile>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'f0) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'f0)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'f1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'f1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'f2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'f2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'f3) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'f3)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'palm) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'palm)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawTactile>)))
  "Returns string type for a message object of type '<RawTactile>"
  "sandia_hand_msgs/RawTactile")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawTactile)))
  "Returns string type for a message object of type 'RawTactile"
  "sandia_hand_msgs/RawTactile")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawTactile>)))
  "Returns md5sum for a message object of type '<RawTactile>"
  "54331defbac2dc0657220f19a2a8c2b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawTactile)))
  "Returns md5sum for a message object of type 'RawTactile"
  "54331defbac2dc0657220f19a2a8c2b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawTactile>)))
  "Returns full string definition for message of type '<RawTactile>"
  (cl:format cl:nil "Header header~%~%uint16[] f0~%uint16[] f1~%uint16[] f2~%uint16[] f3~%uint16[] palm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawTactile)))
  "Returns full string definition for message of type 'RawTactile"
  (cl:format cl:nil "Header header~%~%uint16[] f0~%uint16[] f1~%uint16[] f2~%uint16[] f3~%uint16[] palm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawTactile>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'f0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'f1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'f2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'f3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'palm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawTactile>))
  "Converts a ROS message object to a list"
  (cl:list 'RawTactile
    (cl:cons ':header (header msg))
    (cl:cons ':f0 (f0 msg))
    (cl:cons ':f1 (f1 msg))
    (cl:cons ':f2 (f2 msg))
    (cl:cons ':f3 (f3 msg))
    (cl:cons ':palm (palm msg))
))
