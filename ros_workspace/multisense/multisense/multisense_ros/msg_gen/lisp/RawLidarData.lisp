; Auto-generated. Do not edit!


(cl:in-package multisense_ros-msg)


;//! \htmlinclude RawLidarData.msg.html

(cl:defclass <RawLidarData> (roslisp-msg-protocol:ros-message)
  ((scan_count
    :reader scan_count
    :initarg :scan_count
    :type cl:integer
    :initform 0)
   (time_start
    :reader time_start
    :initarg :time_start
    :type cl:real
    :initform 0)
   (time_end
    :reader time_end
    :initarg :time_end
    :type cl:real
    :initform 0)
   (angle_start
    :reader angle_start
    :initarg :angle_start
    :type cl:integer
    :initform 0)
   (angle_end
    :reader angle_end
    :initarg :angle_end
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (intensity
    :reader intensity
    :initarg :intensity
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass RawLidarData (<RawLidarData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawLidarData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawLidarData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multisense_ros-msg:<RawLidarData> is deprecated: use multisense_ros-msg:RawLidarData instead.")))

(cl:ensure-generic-function 'scan_count-val :lambda-list '(m))
(cl:defmethod scan_count-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:scan_count-val is deprecated.  Use multisense_ros-msg:scan_count instead.")
  (scan_count m))

(cl:ensure-generic-function 'time_start-val :lambda-list '(m))
(cl:defmethod time_start-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:time_start-val is deprecated.  Use multisense_ros-msg:time_start instead.")
  (time_start m))

(cl:ensure-generic-function 'time_end-val :lambda-list '(m))
(cl:defmethod time_end-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:time_end-val is deprecated.  Use multisense_ros-msg:time_end instead.")
  (time_end m))

(cl:ensure-generic-function 'angle_start-val :lambda-list '(m))
(cl:defmethod angle_start-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:angle_start-val is deprecated.  Use multisense_ros-msg:angle_start instead.")
  (angle_start m))

(cl:ensure-generic-function 'angle_end-val :lambda-list '(m))
(cl:defmethod angle_end-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:angle_end-val is deprecated.  Use multisense_ros-msg:angle_end instead.")
  (angle_end m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:distance-val is deprecated.  Use multisense_ros-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <RawLidarData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:intensity-val is deprecated.  Use multisense_ros-msg:intensity instead.")
  (intensity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawLidarData>) ostream)
  "Serializes a message object of type '<RawLidarData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scan_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'scan_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'scan_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'scan_count)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_start)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_start) (cl:floor (cl:slot-value msg 'time_start)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_end)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_end) (cl:floor (cl:slot-value msg 'time_end)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'angle_start)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'angle_end)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'distance))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'intensity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'intensity))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawLidarData>) istream)
  "Deserializes a message object of type '<RawLidarData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scan_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'scan_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'scan_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'scan_count)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_start) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_end) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle_start) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle_end) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'distance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'intensity) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'intensity)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawLidarData>)))
  "Returns string type for a message object of type '<RawLidarData>"
  "multisense_ros/RawLidarData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawLidarData)))
  "Returns string type for a message object of type 'RawLidarData"
  "multisense_ros/RawLidarData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawLidarData>)))
  "Returns md5sum for a message object of type '<RawLidarData>"
  "c6ed0471015a3cddab804db8e53836c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawLidarData)))
  "Returns md5sum for a message object of type 'RawLidarData"
  "c6ed0471015a3cddab804db8e53836c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawLidarData>)))
  "Returns full string definition for message of type '<RawLidarData>"
  (cl:format cl:nil "uint32 scan_count~%time time_start~%time time_end~%int32 angle_start~%int32 angle_end~%uint32[] distance~%uint32[] intensity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawLidarData)))
  "Returns full string definition for message of type 'RawLidarData"
  (cl:format cl:nil "uint32 scan_count~%time time_start~%time time_end~%int32 angle_start~%int32 angle_end~%uint32[] distance~%uint32[] intensity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawLidarData>))
  (cl:+ 0
     4
     8
     8
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'intensity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawLidarData>))
  "Converts a ROS message object to a list"
  (cl:list 'RawLidarData
    (cl:cons ':scan_count (scan_count msg))
    (cl:cons ':time_start (time_start msg))
    (cl:cons ':time_end (time_end msg))
    (cl:cons ':angle_start (angle_start msg))
    (cl:cons ':angle_end (angle_end msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':intensity (intensity msg))
))
