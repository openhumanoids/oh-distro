; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RawFingerInertial.msg.html

(cl:defclass <RawFingerInertial> (roslisp-msg-protocol:ros-message)
  ((mm_accel
    :reader mm_accel
    :initarg :mm_accel
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (pp_accel
    :reader pp_accel
    :initarg :pp_accel
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (dp_accel
    :reader dp_accel
    :initarg :dp_accel
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (mm_mag
    :reader mm_mag
    :initarg :mm_mag
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (pp_mag
    :reader pp_mag
    :initarg :pp_mag
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (dp_mag
    :reader dp_mag
    :initarg :dp_mag
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawFingerInertial (<RawFingerInertial>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawFingerInertial>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawFingerInertial)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RawFingerInertial> is deprecated: use sandia_hand_msgs-msg:RawFingerInertial instead.")))

(cl:ensure-generic-function 'mm_accel-val :lambda-list '(m))
(cl:defmethod mm_accel-val ((m <RawFingerInertial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mm_accel-val is deprecated.  Use sandia_hand_msgs-msg:mm_accel instead.")
  (mm_accel m))

(cl:ensure-generic-function 'pp_accel-val :lambda-list '(m))
(cl:defmethod pp_accel-val ((m <RawFingerInertial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_accel-val is deprecated.  Use sandia_hand_msgs-msg:pp_accel instead.")
  (pp_accel m))

(cl:ensure-generic-function 'dp_accel-val :lambda-list '(m))
(cl:defmethod dp_accel-val ((m <RawFingerInertial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_accel-val is deprecated.  Use sandia_hand_msgs-msg:dp_accel instead.")
  (dp_accel m))

(cl:ensure-generic-function 'mm_mag-val :lambda-list '(m))
(cl:defmethod mm_mag-val ((m <RawFingerInertial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mm_mag-val is deprecated.  Use sandia_hand_msgs-msg:mm_mag instead.")
  (mm_mag m))

(cl:ensure-generic-function 'pp_mag-val :lambda-list '(m))
(cl:defmethod pp_mag-val ((m <RawFingerInertial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_mag-val is deprecated.  Use sandia_hand_msgs-msg:pp_mag instead.")
  (pp_mag m))

(cl:ensure-generic-function 'dp_mag-val :lambda-list '(m))
(cl:defmethod dp_mag-val ((m <RawFingerInertial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_mag-val is deprecated.  Use sandia_hand_msgs-msg:dp_mag instead.")
  (dp_mag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawFingerInertial>) ostream)
  "Serializes a message object of type '<RawFingerInertial>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mm_accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'pp_accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'dp_accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mm_mag))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'pp_mag))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'dp_mag))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawFingerInertial>) istream)
  "Deserializes a message object of type '<RawFingerInertial>"
  (cl:setf (cl:slot-value msg 'mm_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mm_accel)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'pp_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pp_accel)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'dp_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'dp_accel)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'mm_mag) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mm_mag)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'pp_mag) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pp_mag)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'dp_mag) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'dp_mag)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawFingerInertial>)))
  "Returns string type for a message object of type '<RawFingerInertial>"
  "sandia_hand_msgs/RawFingerInertial")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawFingerInertial)))
  "Returns string type for a message object of type 'RawFingerInertial"
  "sandia_hand_msgs/RawFingerInertial")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawFingerInertial>)))
  "Returns md5sum for a message object of type '<RawFingerInertial>"
  "47495f9d3649f45640a29f32e4801b17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawFingerInertial)))
  "Returns md5sum for a message object of type 'RawFingerInertial"
  "47495f9d3649f45640a29f32e4801b17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawFingerInertial>)))
  "Returns full string definition for message of type '<RawFingerInertial>"
  (cl:format cl:nil "uint16[3] mm_accel~%uint16[3] pp_accel~%uint16[3] dp_accel~%uint16[3] mm_mag~%uint16[3] pp_mag~%uint16[3] dp_mag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawFingerInertial)))
  "Returns full string definition for message of type 'RawFingerInertial"
  (cl:format cl:nil "uint16[3] mm_accel~%uint16[3] pp_accel~%uint16[3] dp_accel~%uint16[3] mm_mag~%uint16[3] pp_mag~%uint16[3] dp_mag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawFingerInertial>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mm_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pp_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dp_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mm_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pp_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dp_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawFingerInertial>))
  "Converts a ROS message object to a list"
  (cl:list 'RawFingerInertial
    (cl:cons ':mm_accel (mm_accel msg))
    (cl:cons ':pp_accel (pp_accel msg))
    (cl:cons ':dp_accel (dp_accel msg))
    (cl:cons ':mm_mag (mm_mag msg))
    (cl:cons ':pp_mag (pp_mag msg))
    (cl:cons ':dp_mag (dp_mag msg))
))
