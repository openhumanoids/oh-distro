; Auto-generated. Do not edit!


(cl:in-package handle_msgs-msg)


;//! \htmlinclude HandleControl.msg.html

(cl:defclass <HandleControl> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type (cl:vector cl:integer)
   :initform (cl:make-array 5 :element-type 'cl:integer :initial-element 0))
   (value
    :reader value
    :initarg :value
    :type (cl:vector cl:integer)
   :initform (cl:make-array 5 :element-type 'cl:integer :initial-element 0))
   (valid
    :reader valid
    :initarg :valid
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 5 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass HandleControl (<HandleControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandleControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandleControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handle_msgs-msg:<HandleControl> is deprecated: use handle_msgs-msg:HandleControl instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <HandleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:type-val is deprecated.  Use handle_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <HandleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:value-val is deprecated.  Use handle_msgs-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <HandleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:valid-val is deprecated.  Use handle_msgs-msg:valid instead.")
  (valid m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HandleControl>)))
    "Constants for message type '<HandleControl>"
  '((:VELOCITY . 1)
    (:POSITION . 2)
    (:CURRENT . 3)
    (:VOLTAGE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HandleControl)))
    "Constants for message type 'HandleControl"
  '((:VELOCITY . 1)
    (:POSITION . 2)
    (:CURRENT . 3)
    (:VOLTAGE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandleControl>) ostream)
  "Serializes a message object of type '<HandleControl>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'type))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'value))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'valid))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandleControl>) istream)
  "Deserializes a message object of type '<HandleControl>"
  (cl:setf (cl:slot-value msg 'type) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'type)))
    (cl:dotimes (i 5)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'value) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'value)))
    (cl:dotimes (i 5)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'valid) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'valid)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandleControl>)))
  "Returns string type for a message object of type '<HandleControl>"
  "handle_msgs/HandleControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleControl)))
  "Returns string type for a message object of type 'HandleControl"
  "handle_msgs/HandleControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandleControl>)))
  "Returns md5sum for a message object of type '<HandleControl>"
  "b42b65a1c8b5504458b9cee3c21e3f68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandleControl)))
  "Returns md5sum for a message object of type 'HandleControl"
  "b42b65a1c8b5504458b9cee3c21e3f68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandleControl>)))
  "Returns full string definition for message of type '<HandleControl>"
  (cl:format cl:nil "# Command to move the HANDLE hand~%# ~%~%# The different control types~%uint8 VELOCITY = 1~%uint8 POSITION = 2~%uint8 CURRENT  = 3~%uint8 VOLTAGE  = 4~%~%# The control type for each motor.~%int32[5] type~%~%# The value to set.~%int32[5] value~%~%# Whether or not to control each motor.~%bool[5] valid~%~%# To be added if/when this becomes a service call:~%#---~%#bool ok~%#string reason~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandleControl)))
  "Returns full string definition for message of type 'HandleControl"
  (cl:format cl:nil "# Command to move the HANDLE hand~%# ~%~%# The different control types~%uint8 VELOCITY = 1~%uint8 POSITION = 2~%uint8 CURRENT  = 3~%uint8 VOLTAGE  = 4~%~%# The control type for each motor.~%int32[5] type~%~%# The value to set.~%int32[5] value~%~%# Whether or not to control each motor.~%bool[5] valid~%~%# To be added if/when this becomes a service call:~%#---~%#bool ok~%#string reason~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandleControl>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'type) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'valid) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandleControl>))
  "Converts a ROS message object to a list"
  (cl:list 'HandleControl
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
    (cl:cons ':valid (valid msg))
))
