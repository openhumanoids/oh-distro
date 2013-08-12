; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RawFingerCommands.msg.html

(cl:defclass <RawFingerCommands> (roslisp-msg-protocol:ros-message)
  ((motor_targets
    :reader motor_targets
    :initarg :motor_targets
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawFingerCommands (<RawFingerCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawFingerCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawFingerCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RawFingerCommands> is deprecated: use sandia_hand_msgs-msg:RawFingerCommands instead.")))

(cl:ensure-generic-function 'motor_targets-val :lambda-list '(m))
(cl:defmethod motor_targets-val ((m <RawFingerCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:motor_targets-val is deprecated.  Use sandia_hand_msgs-msg:motor_targets instead.")
  (motor_targets m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawFingerCommands>) ostream)
  "Serializes a message object of type '<RawFingerCommands>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'motor_targets))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawFingerCommands>) istream)
  "Deserializes a message object of type '<RawFingerCommands>"
  (cl:setf (cl:slot-value msg 'motor_targets) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'motor_targets)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawFingerCommands>)))
  "Returns string type for a message object of type '<RawFingerCommands>"
  "sandia_hand_msgs/RawFingerCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawFingerCommands)))
  "Returns string type for a message object of type 'RawFingerCommands"
  "sandia_hand_msgs/RawFingerCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawFingerCommands>)))
  "Returns md5sum for a message object of type '<RawFingerCommands>"
  "62e121b7d21d200f20a78585e63fdbe0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawFingerCommands)))
  "Returns md5sum for a message object of type 'RawFingerCommands"
  "62e121b7d21d200f20a78585e63fdbe0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawFingerCommands>)))
  "Returns full string definition for message of type '<RawFingerCommands>"
  (cl:format cl:nil "int16[3] motor_targets~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawFingerCommands)))
  "Returns full string definition for message of type 'RawFingerCommands"
  (cl:format cl:nil "int16[3] motor_targets~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawFingerCommands>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_targets) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawFingerCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'RawFingerCommands
    (cl:cons ':motor_targets (motor_targets msg))
))
