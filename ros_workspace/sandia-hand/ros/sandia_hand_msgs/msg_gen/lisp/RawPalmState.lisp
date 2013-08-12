; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RawPalmState.msg.html

(cl:defclass <RawPalmState> (roslisp-msg-protocol:ros-message)
  ((palm_time
    :reader palm_time
    :initarg :palm_time
    :type cl:integer
    :initform 0)
   (palm_accel
    :reader palm_accel
    :initarg :palm_accel
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (palm_gyro
    :reader palm_gyro
    :initarg :palm_gyro
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (palm_mag
    :reader palm_mag
    :initarg :palm_mag
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (palm_temps
    :reader palm_temps
    :initarg :palm_temps
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 7 :element-type 'cl:fixnum :initial-element 0))
   (palm_tactile
    :reader palm_tactile
    :initarg :palm_tactile
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 32 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawPalmState (<RawPalmState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawPalmState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawPalmState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RawPalmState> is deprecated: use sandia_hand_msgs-msg:RawPalmState instead.")))

(cl:ensure-generic-function 'palm_time-val :lambda-list '(m))
(cl:defmethod palm_time-val ((m <RawPalmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm_time-val is deprecated.  Use sandia_hand_msgs-msg:palm_time instead.")
  (palm_time m))

(cl:ensure-generic-function 'palm_accel-val :lambda-list '(m))
(cl:defmethod palm_accel-val ((m <RawPalmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm_accel-val is deprecated.  Use sandia_hand_msgs-msg:palm_accel instead.")
  (palm_accel m))

(cl:ensure-generic-function 'palm_gyro-val :lambda-list '(m))
(cl:defmethod palm_gyro-val ((m <RawPalmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm_gyro-val is deprecated.  Use sandia_hand_msgs-msg:palm_gyro instead.")
  (palm_gyro m))

(cl:ensure-generic-function 'palm_mag-val :lambda-list '(m))
(cl:defmethod palm_mag-val ((m <RawPalmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm_mag-val is deprecated.  Use sandia_hand_msgs-msg:palm_mag instead.")
  (palm_mag m))

(cl:ensure-generic-function 'palm_temps-val :lambda-list '(m))
(cl:defmethod palm_temps-val ((m <RawPalmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm_temps-val is deprecated.  Use sandia_hand_msgs-msg:palm_temps instead.")
  (palm_temps m))

(cl:ensure-generic-function 'palm_tactile-val :lambda-list '(m))
(cl:defmethod palm_tactile-val ((m <RawPalmState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:palm_tactile-val is deprecated.  Use sandia_hand_msgs-msg:palm_tactile instead.")
  (palm_tactile m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawPalmState>) ostream)
  "Serializes a message object of type '<RawPalmState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'palm_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'palm_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'palm_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'palm_time)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'palm_accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'palm_gyro))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'palm_mag))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'palm_temps))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'palm_tactile))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawPalmState>) istream)
  "Deserializes a message object of type '<RawPalmState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'palm_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'palm_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'palm_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'palm_time)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'palm_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'palm_accel)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'palm_gyro) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'palm_gyro)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'palm_mag) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'palm_mag)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'palm_temps) (cl:make-array 7))
  (cl:let ((vals (cl:slot-value msg 'palm_temps)))
    (cl:dotimes (i 7)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'palm_tactile) (cl:make-array 32))
  (cl:let ((vals (cl:slot-value msg 'palm_tactile)))
    (cl:dotimes (i 32)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawPalmState>)))
  "Returns string type for a message object of type '<RawPalmState>"
  "sandia_hand_msgs/RawPalmState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawPalmState)))
  "Returns string type for a message object of type 'RawPalmState"
  "sandia_hand_msgs/RawPalmState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawPalmState>)))
  "Returns md5sum for a message object of type '<RawPalmState>"
  "4dac80ef5adde66fbf1e5067579f33bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawPalmState)))
  "Returns md5sum for a message object of type 'RawPalmState"
  "4dac80ef5adde66fbf1e5067579f33bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawPalmState>)))
  "Returns full string definition for message of type '<RawPalmState>"
  (cl:format cl:nil "uint32     palm_time~%int16[3]   palm_accel~%int16[3]   palm_gyro~%int16[3]   palm_mag~%uint16[7]  palm_temps~%uint16[32] palm_tactile~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawPalmState)))
  "Returns full string definition for message of type 'RawPalmState"
  (cl:format cl:nil "uint32     palm_time~%int16[3]   palm_accel~%int16[3]   palm_gyro~%int16[3]   palm_mag~%uint16[7]  palm_temps~%uint16[32] palm_tactile~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawPalmState>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palm_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palm_gyro) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palm_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palm_temps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'palm_tactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawPalmState>))
  "Converts a ROS message object to a list"
  (cl:list 'RawPalmState
    (cl:cons ':palm_time (palm_time msg))
    (cl:cons ':palm_accel (palm_accel msg))
    (cl:cons ':palm_gyro (palm_gyro msg))
    (cl:cons ':palm_mag (palm_mag msg))
    (cl:cons ':palm_temps (palm_temps msg))
    (cl:cons ':palm_tactile (palm_tactile msg))
))
