; Auto-generated. Do not edit!


(cl:in-package handle_msgs-msg)


;//! \htmlinclude Finger.msg.html

(cl:defclass <Finger> (roslisp-msg-protocol:ros-message)
  ((proximal
    :reader proximal
    :initarg :proximal
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (distal
    :reader distal
    :initarg :distal
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Finger (<Finger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Finger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Finger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handle_msgs-msg:<Finger> is deprecated: use handle_msgs-msg:Finger instead.")))

(cl:ensure-generic-function 'proximal-val :lambda-list '(m))
(cl:defmethod proximal-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:proximal-val is deprecated.  Use handle_msgs-msg:proximal instead.")
  (proximal m))

(cl:ensure-generic-function 'distal-val :lambda-list '(m))
(cl:defmethod distal-val ((m <Finger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:distal-val is deprecated.  Use handle_msgs-msg:distal instead.")
  (distal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Finger>) ostream)
  "Serializes a message object of type '<Finger>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'proximal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'proximal))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'distal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Finger>) istream)
  "Deserializes a message object of type '<Finger>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'proximal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'proximal)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'distal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distal)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Finger>)))
  "Returns string type for a message object of type '<Finger>"
  "handle_msgs/Finger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Finger)))
  "Returns string type for a message object of type 'Finger"
  "handle_msgs/Finger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Finger>)))
  "Returns md5sum for a message object of type '<Finger>"
  "143bc4ae61f1c2415fee6b36cd2011b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Finger)))
  "Returns md5sum for a message object of type 'Finger"
  "143bc4ae61f1c2415fee6b36cd2011b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Finger>)))
  "Returns full string definition for message of type '<Finger>"
  (cl:format cl:nil "# This finger definition is used for different sensors in the HandleSensors ~%# message type.  ~%~%float32[] proximal~%float32[] distal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Finger)))
  "Returns full string definition for message of type 'Finger"
  (cl:format cl:nil "# This finger definition is used for different sensors in the HandleSensors ~%# message type.  ~%~%float32[] proximal~%float32[] distal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Finger>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'proximal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Finger>))
  "Converts a ROS message object to a list"
  (cl:list 'Finger
    (cl:cons ':proximal (proximal msg))
    (cl:cons ':distal (distal msg))
))
