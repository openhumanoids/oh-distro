; Auto-generated. Do not edit!


(cl:in-package handle_msgs-msg)


;//! \htmlinclude CableTension.msg.html

(cl:defclass <CableTension> (roslisp-msg-protocol:ros-message)
  ((sensor1
    :reader sensor1
    :initarg :sensor1
    :type cl:float
    :initform 0.0)
   (sensor2
    :reader sensor2
    :initarg :sensor2
    :type cl:float
    :initform 0.0))
)

(cl:defclass CableTension (<CableTension>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CableTension>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CableTension)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handle_msgs-msg:<CableTension> is deprecated: use handle_msgs-msg:CableTension instead.")))

(cl:ensure-generic-function 'sensor1-val :lambda-list '(m))
(cl:defmethod sensor1-val ((m <CableTension>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:sensor1-val is deprecated.  Use handle_msgs-msg:sensor1 instead.")
  (sensor1 m))

(cl:ensure-generic-function 'sensor2-val :lambda-list '(m))
(cl:defmethod sensor2-val ((m <CableTension>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:sensor2-val is deprecated.  Use handle_msgs-msg:sensor2 instead.")
  (sensor2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CableTension>) ostream)
  "Serializes a message object of type '<CableTension>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sensor1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sensor2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CableTension>) istream)
  "Deserializes a message object of type '<CableTension>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sensor1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sensor2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CableTension>)))
  "Returns string type for a message object of type '<CableTension>"
  "handle_msgs/CableTension")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CableTension)))
  "Returns string type for a message object of type 'CableTension"
  "handle_msgs/CableTension")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CableTension>)))
  "Returns md5sum for a message object of type '<CableTension>"
  "b1e553f0fafb9cb708d68c56bd44d521")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CableTension)))
  "Returns md5sum for a message object of type 'CableTension"
  "b1e553f0fafb9cb708d68c56bd44d521")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CableTension>)))
  "Returns full string definition for message of type '<CableTension>"
  (cl:format cl:nil "# The cable tension in one finger of the HANDLE hand.~%~%float32 sensor1~%float32 sensor2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CableTension)))
  "Returns full string definition for message of type 'CableTension"
  (cl:format cl:nil "# The cable tension in one finger of the HANDLE hand.~%~%float32 sensor1~%float32 sensor2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CableTension>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CableTension>))
  "Converts a ROS message object to a list"
  (cl:list 'CableTension
    (cl:cons ':sensor1 (sensor1 msg))
    (cl:cons ':sensor2 (sensor2 msg))
))
