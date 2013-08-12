; Auto-generated. Do not edit!


(cl:in-package handle_msgs-msg)


;//! \htmlinclude HandleSensors.msg.html

(cl:defclass <HandleSensors> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (motorHallEncoder
    :reader motorHallEncoder
    :initarg :motorHallEncoder
    :type (cl:vector cl:integer)
   :initform (cl:make-array 4 :element-type 'cl:integer :initial-element 0))
   (motorWindingTemp
    :reader motorWindingTemp
    :initarg :motorWindingTemp
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (airTemp
    :reader airTemp
    :initarg :airTemp
    :type cl:float
    :initform 0.0)
   (motorVelocity
    :reader motorVelocity
    :initarg :motorVelocity
    :type (cl:vector cl:integer)
   :initform (cl:make-array 4 :element-type 'cl:integer :initial-element 0))
   (motorHousingTemp
    :reader motorHousingTemp
    :initarg :motorHousingTemp
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (motorCurrent
    :reader motorCurrent
    :initarg :motorCurrent
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (fingerTactile
    :reader fingerTactile
    :initarg :fingerTactile
    :type (cl:vector handle_msgs-msg:Finger)
   :initform (cl:make-array 3 :element-type 'handle_msgs-msg:Finger :initial-element (cl:make-instance 'handle_msgs-msg:Finger)))
   (fingerTactileTemp
    :reader fingerTactileTemp
    :initarg :fingerTactileTemp
    :type (cl:vector handle_msgs-msg:Finger)
   :initform (cl:make-array 3 :element-type 'handle_msgs-msg:Finger :initial-element (cl:make-instance 'handle_msgs-msg:Finger)))
   (palmTactile
    :reader palmTactile
    :initarg :palmTactile
    :type (cl:vector cl:float)
   :initform (cl:make-array 48 :element-type 'cl:float :initial-element 0.0))
   (palmTactileTemp
    :reader palmTactileTemp
    :initarg :palmTactileTemp
    :type (cl:vector cl:float)
   :initform (cl:make-array 48 :element-type 'cl:float :initial-element 0.0))
   (fingerSpread
    :reader fingerSpread
    :initarg :fingerSpread
    :type cl:integer
    :initform 0)
   (proximalJointAngle
    :reader proximalJointAngle
    :initarg :proximalJointAngle
    :type (cl:vector cl:integer)
   :initform (cl:make-array 3 :element-type 'cl:integer :initial-element 0))
   (distalJointAngle
    :reader distalJointAngle
    :initarg :distalJointAngle
    :type (cl:vector handle_msgs-msg:Finger)
   :initform (cl:make-array 3 :element-type 'handle_msgs-msg:Finger :initial-element (cl:make-instance 'handle_msgs-msg:Finger)))
   (proximalAcceleration
    :reader proximalAcceleration
    :initarg :proximalAcceleration
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 3 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (distalAcceleration
    :reader distalAcceleration
    :initarg :distalAcceleration
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 3 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3))))
)

(cl:defclass HandleSensors (<HandleSensors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandleSensors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandleSensors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handle_msgs-msg:<HandleSensors> is deprecated: use handle_msgs-msg:HandleSensors instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:header-val is deprecated.  Use handle_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'motorHallEncoder-val :lambda-list '(m))
(cl:defmethod motorHallEncoder-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:motorHallEncoder-val is deprecated.  Use handle_msgs-msg:motorHallEncoder instead.")
  (motorHallEncoder m))

(cl:ensure-generic-function 'motorWindingTemp-val :lambda-list '(m))
(cl:defmethod motorWindingTemp-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:motorWindingTemp-val is deprecated.  Use handle_msgs-msg:motorWindingTemp instead.")
  (motorWindingTemp m))

(cl:ensure-generic-function 'airTemp-val :lambda-list '(m))
(cl:defmethod airTemp-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:airTemp-val is deprecated.  Use handle_msgs-msg:airTemp instead.")
  (airTemp m))

(cl:ensure-generic-function 'motorVelocity-val :lambda-list '(m))
(cl:defmethod motorVelocity-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:motorVelocity-val is deprecated.  Use handle_msgs-msg:motorVelocity instead.")
  (motorVelocity m))

(cl:ensure-generic-function 'motorHousingTemp-val :lambda-list '(m))
(cl:defmethod motorHousingTemp-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:motorHousingTemp-val is deprecated.  Use handle_msgs-msg:motorHousingTemp instead.")
  (motorHousingTemp m))

(cl:ensure-generic-function 'motorCurrent-val :lambda-list '(m))
(cl:defmethod motorCurrent-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:motorCurrent-val is deprecated.  Use handle_msgs-msg:motorCurrent instead.")
  (motorCurrent m))

(cl:ensure-generic-function 'fingerTactile-val :lambda-list '(m))
(cl:defmethod fingerTactile-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:fingerTactile-val is deprecated.  Use handle_msgs-msg:fingerTactile instead.")
  (fingerTactile m))

(cl:ensure-generic-function 'fingerTactileTemp-val :lambda-list '(m))
(cl:defmethod fingerTactileTemp-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:fingerTactileTemp-val is deprecated.  Use handle_msgs-msg:fingerTactileTemp instead.")
  (fingerTactileTemp m))

(cl:ensure-generic-function 'palmTactile-val :lambda-list '(m))
(cl:defmethod palmTactile-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:palmTactile-val is deprecated.  Use handle_msgs-msg:palmTactile instead.")
  (palmTactile m))

(cl:ensure-generic-function 'palmTactileTemp-val :lambda-list '(m))
(cl:defmethod palmTactileTemp-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:palmTactileTemp-val is deprecated.  Use handle_msgs-msg:palmTactileTemp instead.")
  (palmTactileTemp m))

(cl:ensure-generic-function 'fingerSpread-val :lambda-list '(m))
(cl:defmethod fingerSpread-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:fingerSpread-val is deprecated.  Use handle_msgs-msg:fingerSpread instead.")
  (fingerSpread m))

(cl:ensure-generic-function 'proximalJointAngle-val :lambda-list '(m))
(cl:defmethod proximalJointAngle-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:proximalJointAngle-val is deprecated.  Use handle_msgs-msg:proximalJointAngle instead.")
  (proximalJointAngle m))

(cl:ensure-generic-function 'distalJointAngle-val :lambda-list '(m))
(cl:defmethod distalJointAngle-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:distalJointAngle-val is deprecated.  Use handle_msgs-msg:distalJointAngle instead.")
  (distalJointAngle m))

(cl:ensure-generic-function 'proximalAcceleration-val :lambda-list '(m))
(cl:defmethod proximalAcceleration-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:proximalAcceleration-val is deprecated.  Use handle_msgs-msg:proximalAcceleration instead.")
  (proximalAcceleration m))

(cl:ensure-generic-function 'distalAcceleration-val :lambda-list '(m))
(cl:defmethod distalAcceleration-val ((m <HandleSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:distalAcceleration-val is deprecated.  Use handle_msgs-msg:distalAcceleration instead.")
  (distalAcceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandleSensors>) ostream)
  "Serializes a message object of type '<HandleSensors>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'motorHallEncoder))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'motorWindingTemp))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'airTemp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'motorVelocity))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'motorHousingTemp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'motorCurrent))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'fingerTactile))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'fingerTactileTemp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'palmTactile))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'palmTactileTemp))
  (cl:let* ((signed (cl:slot-value msg 'fingerSpread)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'proximalJointAngle))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'distalJointAngle))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'proximalAcceleration))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'distalAcceleration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandleSensors>) istream)
  "Deserializes a message object of type '<HandleSensors>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'motorHallEncoder) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'motorHallEncoder)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'motorWindingTemp) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'motorWindingTemp)))
    (cl:dotimes (i 4)
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
    (cl:setf (cl:slot-value msg 'airTemp) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'motorVelocity) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'motorVelocity)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'motorHousingTemp) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'motorHousingTemp)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'motorCurrent) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'motorCurrent)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'fingerTactile) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'fingerTactile)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'handle_msgs-msg:Finger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'fingerTactileTemp) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'fingerTactileTemp)))
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
  (cl:setf (cl:slot-value msg 'palmTactileTemp) (cl:make-array 48))
  (cl:let ((vals (cl:slot-value msg 'palmTactileTemp)))
    (cl:dotimes (i 48)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fingerSpread) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'proximalJointAngle) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'proximalJointAngle)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'distalJointAngle) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'distalJointAngle)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'handle_msgs-msg:Finger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'proximalAcceleration) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'proximalAcceleration)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'distalAcceleration) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'distalAcceleration)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandleSensors>)))
  "Returns string type for a message object of type '<HandleSensors>"
  "handle_msgs/HandleSensors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleSensors)))
  "Returns string type for a message object of type 'HandleSensors"
  "handle_msgs/HandleSensors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandleSensors>)))
  "Returns md5sum for a message object of type '<HandleSensors>"
  "4f61f30841dadd6b89d33ed7dc93ef10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandleSensors)))
  "Returns md5sum for a message object of type 'HandleSensors"
  "4f61f30841dadd6b89d33ed7dc93ef10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandleSensors>)))
  "Returns full string definition for message of type '<HandleSensors>"
  (cl:format cl:nil "# All of the sensors on the HANDLE hand.~%#~%# NOTE: In general, the order of the arrays is: [F1, F2, F3, F3 Ant., Spread].~%# Where: F1 is analogous to your right hand index finger, F2 is analogous to ~%# your right middle finger, F3 is analogous to your right thumb, F3 Ant. is ~%# the antagonistic motor for F3, and Spread is the motor which controls the ~%# rotation of F1 and F2.~%#~%~%# Currently only used for time stamp.  Time stamp set from Overo's clock.~%# Not guaranteed to be in sync with the current time.~%Header header~%~%# The hall effect sensor on the finger motors.  ~%# 24 counts per motor revolution x 300 motor revolutions for one full spindle~%# rotation.  ~%# 3500 to put finger at approx. 90 degrees~%# 6000 to close finger gently~%# 7000 to close finger tightly~%#~%# [F1, F2, F3, F3 Ant.]~%int32[4] motorHallEncoder~%~%# The temperature of the finger motor windings, in Celsius.~%#~%# [F1, F2, F3, F3 Ant.]~%float32[4] motorWindingTemp~%~%# The air temperature as measured inside the housing, in Celsius.~%float32 airTemp~%~%# The motor velocity in RPM.  (Hall encoder ticks per minute)~%#~%# [F1, F2, F3, F3 Ant.]~%int32[4] motorVelocity~%~%# The temperature of the motor housing, in Celsius.~%# Note that the housing temp for the Spread motor is not populated, so will~%# not have a logical value.~%#~%# [F1, F2, F3, F3 Ant., Spread]~%float32[5] motorHousingTemp~%~%# The motor current in amps.~%#~%# [F1, F2, F3, F3 Ant., Spread]~%float32[5] motorCurrent~%~%# The tactile array for each finger.  In units of ADC counts.~%# Note there are 12 proximal and 10 distal sensors.~%#~%# [F1, F2, F3]~%Finger[3] fingerTactile~%~%# The tactile temperature array for each finger, in Celsius.~%# Note there are 12 proximal and 10 distal sensors.~%#~%# [F1, F2, F3]~%Finger[3] fingerTactileTemp~%~%# The tactile array for the palm.  In units of ADC counts.~%float32[48] palmTactile~%~%# The tactile temperature array for the palm, in Celsius.~%float32[48] palmTactileTemp~%~%# The encoder on the F1 / F2 rotation. 8.533 ticks per degree.~%# 3072 ticks per 180 degree rotation. (not possible).~%# 768 ticks to rotate the fingers 90 degrees for a \"T\" grasp.~%# 512 ticks to rotate the fingers 60 degrees for a spherical grasp.~%int32 fingerSpread~%~%# The proximal joint angle.  Approx 2.84 ticks per degree.~%# 1024 ticks per full revolution. (not possible)~%# 256 ticks to put finger at approx. 90 degrees~%# 435 ticks to close finger gently~%# 445 ticks to close finger tightly~%#~%# [F1, F2, F3]~%int32[3] proximalJointAngle~%~%# The finger distal joint flexture angle~%# [F1, F2, F3]~%# Note there are 4 readings on either side of the joint.~%Finger[3] distalJointAngle~%~%# The accelerometer in the proximal link of each finger.  In G's.~%# This sensor is not populated at this time.~%#~%# [F1, F2, F3]~%geometry_msgs/Vector3[3] proximalAcceleration~%~%# The accelerometer in the distal link of each finger.  In G's.~%# Z axis points out the back of the finger.~%# Y axis points out the finger tip.~%# X axis points to the left when looking at the finger pad.~%#~%# [F1, F2, F3]~%geometry_msgs/Vector3[3] distalAcceleration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: handle_msgs/Finger~%# This finger definition is used for different sensors in the HandleSensors ~%# message type.  ~%~%float32[] proximal~%float32[] distal~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandleSensors)))
  "Returns full string definition for message of type 'HandleSensors"
  (cl:format cl:nil "# All of the sensors on the HANDLE hand.~%#~%# NOTE: In general, the order of the arrays is: [F1, F2, F3, F3 Ant., Spread].~%# Where: F1 is analogous to your right hand index finger, F2 is analogous to ~%# your right middle finger, F3 is analogous to your right thumb, F3 Ant. is ~%# the antagonistic motor for F3, and Spread is the motor which controls the ~%# rotation of F1 and F2.~%#~%~%# Currently only used for time stamp.  Time stamp set from Overo's clock.~%# Not guaranteed to be in sync with the current time.~%Header header~%~%# The hall effect sensor on the finger motors.  ~%# 24 counts per motor revolution x 300 motor revolutions for one full spindle~%# rotation.  ~%# 3500 to put finger at approx. 90 degrees~%# 6000 to close finger gently~%# 7000 to close finger tightly~%#~%# [F1, F2, F3, F3 Ant.]~%int32[4] motorHallEncoder~%~%# The temperature of the finger motor windings, in Celsius.~%#~%# [F1, F2, F3, F3 Ant.]~%float32[4] motorWindingTemp~%~%# The air temperature as measured inside the housing, in Celsius.~%float32 airTemp~%~%# The motor velocity in RPM.  (Hall encoder ticks per minute)~%#~%# [F1, F2, F3, F3 Ant.]~%int32[4] motorVelocity~%~%# The temperature of the motor housing, in Celsius.~%# Note that the housing temp for the Spread motor is not populated, so will~%# not have a logical value.~%#~%# [F1, F2, F3, F3 Ant., Spread]~%float32[5] motorHousingTemp~%~%# The motor current in amps.~%#~%# [F1, F2, F3, F3 Ant., Spread]~%float32[5] motorCurrent~%~%# The tactile array for each finger.  In units of ADC counts.~%# Note there are 12 proximal and 10 distal sensors.~%#~%# [F1, F2, F3]~%Finger[3] fingerTactile~%~%# The tactile temperature array for each finger, in Celsius.~%# Note there are 12 proximal and 10 distal sensors.~%#~%# [F1, F2, F3]~%Finger[3] fingerTactileTemp~%~%# The tactile array for the palm.  In units of ADC counts.~%float32[48] palmTactile~%~%# The tactile temperature array for the palm, in Celsius.~%float32[48] palmTactileTemp~%~%# The encoder on the F1 / F2 rotation. 8.533 ticks per degree.~%# 3072 ticks per 180 degree rotation. (not possible).~%# 768 ticks to rotate the fingers 90 degrees for a \"T\" grasp.~%# 512 ticks to rotate the fingers 60 degrees for a spherical grasp.~%int32 fingerSpread~%~%# The proximal joint angle.  Approx 2.84 ticks per degree.~%# 1024 ticks per full revolution. (not possible)~%# 256 ticks to put finger at approx. 90 degrees~%# 435 ticks to close finger gently~%# 445 ticks to close finger tightly~%#~%# [F1, F2, F3]~%int32[3] proximalJointAngle~%~%# The finger distal joint flexture angle~%# [F1, F2, F3]~%# Note there are 4 readings on either side of the joint.~%Finger[3] distalJointAngle~%~%# The accelerometer in the proximal link of each finger.  In G's.~%# This sensor is not populated at this time.~%#~%# [F1, F2, F3]~%geometry_msgs/Vector3[3] proximalAcceleration~%~%# The accelerometer in the distal link of each finger.  In G's.~%# Z axis points out the back of the finger.~%# Y axis points out the finger tip.~%# X axis points to the left when looking at the finger pad.~%#~%# [F1, F2, F3]~%geometry_msgs/Vector3[3] distalAcceleration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: handle_msgs/Finger~%# This finger definition is used for different sensors in the HandleSensors ~%# message type.  ~%~%float32[] proximal~%float32[] distal~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandleSensors>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorHallEncoder) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorWindingTemp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorVelocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorHousingTemp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorCurrent) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fingerTactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fingerTactileTemp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palmTactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palmTactileTemp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'proximalJointAngle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'distalJointAngle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'proximalAcceleration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'distalAcceleration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandleSensors>))
  "Converts a ROS message object to a list"
  (cl:list 'HandleSensors
    (cl:cons ':header (header msg))
    (cl:cons ':motorHallEncoder (motorHallEncoder msg))
    (cl:cons ':motorWindingTemp (motorWindingTemp msg))
    (cl:cons ':airTemp (airTemp msg))
    (cl:cons ':motorVelocity (motorVelocity msg))
    (cl:cons ':motorHousingTemp (motorHousingTemp msg))
    (cl:cons ':motorCurrent (motorCurrent msg))
    (cl:cons ':fingerTactile (fingerTactile msg))
    (cl:cons ':fingerTactileTemp (fingerTactileTemp msg))
    (cl:cons ':palmTactile (palmTactile msg))
    (cl:cons ':palmTactileTemp (palmTactileTemp msg))
    (cl:cons ':fingerSpread (fingerSpread msg))
    (cl:cons ':proximalJointAngle (proximalJointAngle msg))
    (cl:cons ':distalJointAngle (distalJointAngle msg))
    (cl:cons ':proximalAcceleration (proximalAcceleration msg))
    (cl:cons ':distalAcceleration (distalAcceleration msg))
))
