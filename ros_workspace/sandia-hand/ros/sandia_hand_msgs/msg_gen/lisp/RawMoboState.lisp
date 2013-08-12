; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RawMoboState.msg.html

(cl:defclass <RawMoboState> (roslisp-msg-protocol:ros-message)
  ((mobo_time
    :reader mobo_time
    :initarg :mobo_time
    :type cl:integer
    :initform 0)
   (finger_currents
    :reader finger_currents
    :initarg :finger_currents
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (logic_currents
    :reader logic_currents
    :initarg :logic_currents
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mobo_temp
    :reader mobo_temp
    :initarg :mobo_temp
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (mobo_max_effort
    :reader mobo_max_effort
    :initarg :mobo_max_effort
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RawMoboState (<RawMoboState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawMoboState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawMoboState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RawMoboState> is deprecated: use sandia_hand_msgs-msg:RawMoboState instead.")))

(cl:ensure-generic-function 'mobo_time-val :lambda-list '(m))
(cl:defmethod mobo_time-val ((m <RawMoboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mobo_time-val is deprecated.  Use sandia_hand_msgs-msg:mobo_time instead.")
  (mobo_time m))

(cl:ensure-generic-function 'finger_currents-val :lambda-list '(m))
(cl:defmethod finger_currents-val ((m <RawMoboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:finger_currents-val is deprecated.  Use sandia_hand_msgs-msg:finger_currents instead.")
  (finger_currents m))

(cl:ensure-generic-function 'logic_currents-val :lambda-list '(m))
(cl:defmethod logic_currents-val ((m <RawMoboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:logic_currents-val is deprecated.  Use sandia_hand_msgs-msg:logic_currents instead.")
  (logic_currents m))

(cl:ensure-generic-function 'mobo_temp-val :lambda-list '(m))
(cl:defmethod mobo_temp-val ((m <RawMoboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mobo_temp-val is deprecated.  Use sandia_hand_msgs-msg:mobo_temp instead.")
  (mobo_temp m))

(cl:ensure-generic-function 'mobo_max_effort-val :lambda-list '(m))
(cl:defmethod mobo_max_effort-val ((m <RawMoboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mobo_max_effort-val is deprecated.  Use sandia_hand_msgs-msg:mobo_max_effort instead.")
  (mobo_max_effort m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawMoboState>) ostream)
  "Serializes a message object of type '<RawMoboState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mobo_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mobo_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mobo_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mobo_time)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'finger_currents))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'logic_currents))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mobo_temp))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mobo_max_effort)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawMoboState>) istream)
  "Deserializes a message object of type '<RawMoboState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mobo_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mobo_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mobo_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mobo_time)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'finger_currents) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'finger_currents)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'logic_currents) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'logic_currents)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'mobo_temp) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mobo_temp)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mobo_max_effort)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawMoboState>)))
  "Returns string type for a message object of type '<RawMoboState>"
  "sandia_hand_msgs/RawMoboState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawMoboState)))
  "Returns string type for a message object of type 'RawMoboState"
  "sandia_hand_msgs/RawMoboState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawMoboState>)))
  "Returns md5sum for a message object of type '<RawMoboState>"
  "4e7e33569ea3b39428acbd57e6ef85b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawMoboState)))
  "Returns md5sum for a message object of type 'RawMoboState"
  "4e7e33569ea3b39428acbd57e6ef85b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawMoboState>)))
  "Returns full string definition for message of type '<RawMoboState>"
  (cl:format cl:nil "uint32 mobo_time~%float32[4] finger_currents~%float32[3] logic_currents~%uint16[3] mobo_temp~%uint8 mobo_max_effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawMoboState)))
  "Returns full string definition for message of type 'RawMoboState"
  (cl:format cl:nil "uint32 mobo_time~%float32[4] finger_currents~%float32[3] logic_currents~%uint16[3] mobo_temp~%uint8 mobo_max_effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawMoboState>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger_currents) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'logic_currents) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mobo_temp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawMoboState>))
  "Converts a ROS message object to a list"
  (cl:list 'RawMoboState
    (cl:cons ':mobo_time (mobo_time msg))
    (cl:cons ':finger_currents (finger_currents msg))
    (cl:cons ':logic_currents (logic_currents msg))
    (cl:cons ':mobo_temp (mobo_temp msg))
    (cl:cons ':mobo_max_effort (mobo_max_effort msg))
))
