; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude RawFingerState.msg.html

(cl:defclass <RawFingerState> (roslisp-msg-protocol:ros-message)
  ((fmcb_time
    :reader fmcb_time
    :initarg :fmcb_time
    :type cl:integer
    :initform 0)
   (pp_time
    :reader pp_time
    :initarg :pp_time
    :type cl:integer
    :initform 0)
   (dp_time
    :reader dp_time
    :initarg :dp_time
    :type cl:integer
    :initform 0)
   (pp_tactile
    :reader pp_tactile
    :initarg :pp_tactile
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (dp_tactile
    :reader dp_tactile
    :initarg :dp_tactile
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 12 :element-type 'cl:fixnum :initial-element 0))
   (pp_strain
    :reader pp_strain
    :initarg :pp_strain
    :type cl:integer
    :initform 0)
   (mm_accel
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
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (pp_temp
    :reader pp_temp
    :initarg :pp_temp
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (dp_temp
    :reader dp_temp
    :initarg :dp_temp
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (fmcb_temp
    :reader fmcb_temp
    :initarg :fmcb_temp
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (fmcb_voltage
    :reader fmcb_voltage
    :initarg :fmcb_voltage
    :type cl:fixnum
    :initform 0)
   (fmcb_pb_current
    :reader fmcb_pb_current
    :initarg :fmcb_pb_current
    :type cl:fixnum
    :initform 0)
   (hall_tgt
    :reader hall_tgt
    :initarg :hall_tgt
    :type (cl:vector cl:integer)
   :initform (cl:make-array 3 :element-type 'cl:integer :initial-element 0))
   (hall_pos
    :reader hall_pos
    :initarg :hall_pos
    :type (cl:vector cl:integer)
   :initform (cl:make-array 3 :element-type 'cl:integer :initial-element 0))
   (fmcb_effort
    :reader fmcb_effort
    :initarg :fmcb_effort
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawFingerState (<RawFingerState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawFingerState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawFingerState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<RawFingerState> is deprecated: use sandia_hand_msgs-msg:RawFingerState instead.")))

(cl:ensure-generic-function 'fmcb_time-val :lambda-list '(m))
(cl:defmethod fmcb_time-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:fmcb_time-val is deprecated.  Use sandia_hand_msgs-msg:fmcb_time instead.")
  (fmcb_time m))

(cl:ensure-generic-function 'pp_time-val :lambda-list '(m))
(cl:defmethod pp_time-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_time-val is deprecated.  Use sandia_hand_msgs-msg:pp_time instead.")
  (pp_time m))

(cl:ensure-generic-function 'dp_time-val :lambda-list '(m))
(cl:defmethod dp_time-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_time-val is deprecated.  Use sandia_hand_msgs-msg:dp_time instead.")
  (dp_time m))

(cl:ensure-generic-function 'pp_tactile-val :lambda-list '(m))
(cl:defmethod pp_tactile-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_tactile-val is deprecated.  Use sandia_hand_msgs-msg:pp_tactile instead.")
  (pp_tactile m))

(cl:ensure-generic-function 'dp_tactile-val :lambda-list '(m))
(cl:defmethod dp_tactile-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_tactile-val is deprecated.  Use sandia_hand_msgs-msg:dp_tactile instead.")
  (dp_tactile m))

(cl:ensure-generic-function 'pp_strain-val :lambda-list '(m))
(cl:defmethod pp_strain-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_strain-val is deprecated.  Use sandia_hand_msgs-msg:pp_strain instead.")
  (pp_strain m))

(cl:ensure-generic-function 'mm_accel-val :lambda-list '(m))
(cl:defmethod mm_accel-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mm_accel-val is deprecated.  Use sandia_hand_msgs-msg:mm_accel instead.")
  (mm_accel m))

(cl:ensure-generic-function 'pp_accel-val :lambda-list '(m))
(cl:defmethod pp_accel-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_accel-val is deprecated.  Use sandia_hand_msgs-msg:pp_accel instead.")
  (pp_accel m))

(cl:ensure-generic-function 'dp_accel-val :lambda-list '(m))
(cl:defmethod dp_accel-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_accel-val is deprecated.  Use sandia_hand_msgs-msg:dp_accel instead.")
  (dp_accel m))

(cl:ensure-generic-function 'mm_mag-val :lambda-list '(m))
(cl:defmethod mm_mag-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:mm_mag-val is deprecated.  Use sandia_hand_msgs-msg:mm_mag instead.")
  (mm_mag m))

(cl:ensure-generic-function 'pp_mag-val :lambda-list '(m))
(cl:defmethod pp_mag-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_mag-val is deprecated.  Use sandia_hand_msgs-msg:pp_mag instead.")
  (pp_mag m))

(cl:ensure-generic-function 'dp_mag-val :lambda-list '(m))
(cl:defmethod dp_mag-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_mag-val is deprecated.  Use sandia_hand_msgs-msg:dp_mag instead.")
  (dp_mag m))

(cl:ensure-generic-function 'pp_temp-val :lambda-list '(m))
(cl:defmethod pp_temp-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:pp_temp-val is deprecated.  Use sandia_hand_msgs-msg:pp_temp instead.")
  (pp_temp m))

(cl:ensure-generic-function 'dp_temp-val :lambda-list '(m))
(cl:defmethod dp_temp-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:dp_temp-val is deprecated.  Use sandia_hand_msgs-msg:dp_temp instead.")
  (dp_temp m))

(cl:ensure-generic-function 'fmcb_temp-val :lambda-list '(m))
(cl:defmethod fmcb_temp-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:fmcb_temp-val is deprecated.  Use sandia_hand_msgs-msg:fmcb_temp instead.")
  (fmcb_temp m))

(cl:ensure-generic-function 'fmcb_voltage-val :lambda-list '(m))
(cl:defmethod fmcb_voltage-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:fmcb_voltage-val is deprecated.  Use sandia_hand_msgs-msg:fmcb_voltage instead.")
  (fmcb_voltage m))

(cl:ensure-generic-function 'fmcb_pb_current-val :lambda-list '(m))
(cl:defmethod fmcb_pb_current-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:fmcb_pb_current-val is deprecated.  Use sandia_hand_msgs-msg:fmcb_pb_current instead.")
  (fmcb_pb_current m))

(cl:ensure-generic-function 'hall_tgt-val :lambda-list '(m))
(cl:defmethod hall_tgt-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:hall_tgt-val is deprecated.  Use sandia_hand_msgs-msg:hall_tgt instead.")
  (hall_tgt m))

(cl:ensure-generic-function 'hall_pos-val :lambda-list '(m))
(cl:defmethod hall_pos-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:hall_pos-val is deprecated.  Use sandia_hand_msgs-msg:hall_pos instead.")
  (hall_pos m))

(cl:ensure-generic-function 'fmcb_effort-val :lambda-list '(m))
(cl:defmethod fmcb_effort-val ((m <RawFingerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:fmcb_effort-val is deprecated.  Use sandia_hand_msgs-msg:fmcb_effort instead.")
  (fmcb_effort m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawFingerState>) ostream)
  "Serializes a message object of type '<RawFingerState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fmcb_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fmcb_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fmcb_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fmcb_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dp_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dp_time)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'pp_tactile))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'dp_tactile))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pp_strain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pp_strain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pp_strain)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pp_strain)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mm_accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'pp_accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'pp_temp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'dp_temp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'fmcb_temp))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fmcb_voltage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fmcb_voltage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fmcb_pb_current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fmcb_pb_current)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'hall_tgt))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'hall_pos))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'fmcb_effort))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawFingerState>) istream)
  "Deserializes a message object of type '<RawFingerState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fmcb_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fmcb_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fmcb_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fmcb_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dp_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dp_time)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pp_tactile) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pp_tactile)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'dp_tactile) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'dp_tactile)))
    (cl:dotimes (i 12)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pp_strain)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pp_strain)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pp_strain)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pp_strain)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mm_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'mm_accel)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'pp_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pp_accel)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'dp_accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'dp_accel)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
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
  (cl:setf (cl:slot-value msg 'pp_temp) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pp_temp)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'dp_temp) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'dp_temp)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'fmcb_temp) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'fmcb_temp)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fmcb_voltage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fmcb_voltage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fmcb_pb_current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fmcb_pb_current)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'hall_tgt) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'hall_tgt)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'hall_pos) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'hall_pos)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'fmcb_effort) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'fmcb_effort)))
    (cl:dotimes (i 3)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawFingerState>)))
  "Returns string type for a message object of type '<RawFingerState>"
  "sandia_hand_msgs/RawFingerState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawFingerState)))
  "Returns string type for a message object of type 'RawFingerState"
  "sandia_hand_msgs/RawFingerState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawFingerState>)))
  "Returns md5sum for a message object of type '<RawFingerState>"
  "a2c29ce00f185a2ceab39770dc9bc042")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawFingerState)))
  "Returns md5sum for a message object of type 'RawFingerState"
  "a2c29ce00f185a2ceab39770dc9bc042")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawFingerState>)))
  "Returns full string definition for message of type '<RawFingerState>"
  (cl:format cl:nil "uint32 fmcb_time~%uint32 pp_time~%uint32 dp_time~%uint16[6]  pp_tactile~%uint16[12] dp_tactile~%uint32 pp_strain~%int16[3] mm_accel~%int16[3] pp_accel~%int16[3] dp_accel~%uint16[3] mm_mag~%uint16[3] pp_mag~%uint16[3] dp_mag~%uint16[4] pp_temp~%uint16[4] dp_temp~%uint16[3] fmcb_temp~%uint16 fmcb_voltage~%uint16 fmcb_pb_current~%int32[3] hall_tgt~%int32[3] hall_pos~%int16[3] fmcb_effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawFingerState)))
  "Returns full string definition for message of type 'RawFingerState"
  (cl:format cl:nil "uint32 fmcb_time~%uint32 pp_time~%uint32 dp_time~%uint16[6]  pp_tactile~%uint16[12] dp_tactile~%uint32 pp_strain~%int16[3] mm_accel~%int16[3] pp_accel~%int16[3] dp_accel~%uint16[3] mm_mag~%uint16[3] pp_mag~%uint16[3] dp_mag~%uint16[4] pp_temp~%uint16[4] dp_temp~%uint16[3] fmcb_temp~%uint16 fmcb_voltage~%uint16 fmcb_pb_current~%int32[3] hall_tgt~%int32[3] hall_pos~%int16[3] fmcb_effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawFingerState>))
  (cl:+ 0
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pp_tactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dp_tactile) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mm_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pp_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dp_accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mm_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pp_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dp_mag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pp_temp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dp_temp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fmcb_temp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     2
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'hall_tgt) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'hall_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fmcb_effort) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawFingerState>))
  "Converts a ROS message object to a list"
  (cl:list 'RawFingerState
    (cl:cons ':fmcb_time (fmcb_time msg))
    (cl:cons ':pp_time (pp_time msg))
    (cl:cons ':dp_time (dp_time msg))
    (cl:cons ':pp_tactile (pp_tactile msg))
    (cl:cons ':dp_tactile (dp_tactile msg))
    (cl:cons ':pp_strain (pp_strain msg))
    (cl:cons ':mm_accel (mm_accel msg))
    (cl:cons ':pp_accel (pp_accel msg))
    (cl:cons ':dp_accel (dp_accel msg))
    (cl:cons ':mm_mag (mm_mag msg))
    (cl:cons ':pp_mag (pp_mag msg))
    (cl:cons ':dp_mag (dp_mag msg))
    (cl:cons ':pp_temp (pp_temp msg))
    (cl:cons ':dp_temp (dp_temp msg))
    (cl:cons ':fmcb_temp (fmcb_temp msg))
    (cl:cons ':fmcb_voltage (fmcb_voltage msg))
    (cl:cons ':fmcb_pb_current (fmcb_pb_current msg))
    (cl:cons ':hall_tgt (hall_tgt msg))
    (cl:cons ':hall_pos (hall_pos msg))
    (cl:cons ':fmcb_effort (fmcb_effort msg))
))
