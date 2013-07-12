; Auto-generated. Do not edit!


(cl:in-package multisense_ros-msg)


;//! \htmlinclude RawCamData.msg.html

(cl:defclass <RawCamData> (roslisp-msg-protocol:ros-message)
  ((frames_per_second
    :reader frames_per_second
    :initarg :frames_per_second
    :type cl:float
    :initform 0.0)
   (gain
    :reader gain
    :initarg :gain
    :type cl:float
    :initform 0.0)
   (exposure_time
    :reader exposure_time
    :initarg :exposure_time
    :type cl:integer
    :initform 0)
   (frame_count
    :reader frame_count
    :initarg :frame_count
    :type cl:integer
    :initform 0)
   (time_stamp
    :reader time_stamp
    :initarg :time_stamp
    :type cl:real
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0)
   (gray_scale_image
    :reader gray_scale_image
    :initarg :gray_scale_image
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (disparity_image
    :reader disparity_image
    :initarg :disparity_image
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RawCamData (<RawCamData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawCamData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawCamData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multisense_ros-msg:<RawCamData> is deprecated: use multisense_ros-msg:RawCamData instead.")))

(cl:ensure-generic-function 'frames_per_second-val :lambda-list '(m))
(cl:defmethod frames_per_second-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:frames_per_second-val is deprecated.  Use multisense_ros-msg:frames_per_second instead.")
  (frames_per_second m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:gain-val is deprecated.  Use multisense_ros-msg:gain instead.")
  (gain m))

(cl:ensure-generic-function 'exposure_time-val :lambda-list '(m))
(cl:defmethod exposure_time-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:exposure_time-val is deprecated.  Use multisense_ros-msg:exposure_time instead.")
  (exposure_time m))

(cl:ensure-generic-function 'frame_count-val :lambda-list '(m))
(cl:defmethod frame_count-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:frame_count-val is deprecated.  Use multisense_ros-msg:frame_count instead.")
  (frame_count m))

(cl:ensure-generic-function 'time_stamp-val :lambda-list '(m))
(cl:defmethod time_stamp-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:time_stamp-val is deprecated.  Use multisense_ros-msg:time_stamp instead.")
  (time_stamp m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:angle-val is deprecated.  Use multisense_ros-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:width-val is deprecated.  Use multisense_ros-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:height-val is deprecated.  Use multisense_ros-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'gray_scale_image-val :lambda-list '(m))
(cl:defmethod gray_scale_image-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:gray_scale_image-val is deprecated.  Use multisense_ros-msg:gray_scale_image instead.")
  (gray_scale_image m))

(cl:ensure-generic-function 'disparity_image-val :lambda-list '(m))
(cl:defmethod disparity_image-val ((m <RawCamData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multisense_ros-msg:disparity_image-val is deprecated.  Use multisense_ros-msg:disparity_image instead.")
  (disparity_image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawCamData>) ostream)
  "Serializes a message object of type '<RawCamData>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frames_per_second))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'exposure_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'exposure_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'exposure_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'exposure_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_count)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_stamp) (cl:floor (cl:slot-value msg 'time_stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gray_scale_image))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'gray_scale_image))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'disparity_image))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'disparity_image))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawCamData>) istream)
  "Deserializes a message object of type '<RawCamData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frames_per_second) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'exposure_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'exposure_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'exposure_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'exposure_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_count)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gray_scale_image) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gray_scale_image)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'disparity_image) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'disparity_image)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawCamData>)))
  "Returns string type for a message object of type '<RawCamData>"
  "multisense_ros/RawCamData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawCamData)))
  "Returns string type for a message object of type 'RawCamData"
  "multisense_ros/RawCamData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawCamData>)))
  "Returns md5sum for a message object of type '<RawCamData>"
  "5088c19778d4fa49ece5e07c0880c7e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawCamData)))
  "Returns md5sum for a message object of type 'RawCamData"
  "5088c19778d4fa49ece5e07c0880c7e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawCamData>)))
  "Returns full string definition for message of type '<RawCamData>"
  (cl:format cl:nil "float32 frames_per_second~%float32 gain~%uint32  exposure_time~%uint32  frame_count~%time    time_stamp~%uint32  angle~%uint16  width~%uint16  height~%uint8[] gray_scale_image~%uint16[] disparity_image~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawCamData)))
  "Returns full string definition for message of type 'RawCamData"
  (cl:format cl:nil "float32 frames_per_second~%float32 gain~%uint32  exposure_time~%uint32  frame_count~%time    time_stamp~%uint32  angle~%uint16  width~%uint16  height~%uint8[] gray_scale_image~%uint16[] disparity_image~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawCamData>))
  (cl:+ 0
     4
     4
     4
     4
     8
     4
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gray_scale_image) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'disparity_image) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawCamData>))
  "Converts a ROS message object to a list"
  (cl:list 'RawCamData
    (cl:cons ':frames_per_second (frames_per_second msg))
    (cl:cons ':gain (gain msg))
    (cl:cons ':exposure_time (exposure_time msg))
    (cl:cons ':frame_count (frame_count msg))
    (cl:cons ':time_stamp (time_stamp msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':gray_scale_image (gray_scale_image msg))
    (cl:cons ':disparity_image (disparity_image msg))
))
