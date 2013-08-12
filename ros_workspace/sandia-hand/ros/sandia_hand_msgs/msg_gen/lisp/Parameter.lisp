; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude Parameter.msg.html

(cl:defclass <Parameter> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (val_type
    :reader val_type
    :initarg :val_type
    :type cl:integer
    :initform 0)
   (i_val
    :reader i_val
    :initarg :i_val
    :type cl:integer
    :initform 0)
   (f_val
    :reader f_val
    :initarg :f_val
    :type cl:float
    :initform 0.0))
)

(cl:defclass Parameter (<Parameter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Parameter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Parameter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<Parameter> is deprecated: use sandia_hand_msgs-msg:Parameter instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Parameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:name-val is deprecated.  Use sandia_hand_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'val_type-val :lambda-list '(m))
(cl:defmethod val_type-val ((m <Parameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:val_type-val is deprecated.  Use sandia_hand_msgs-msg:val_type instead.")
  (val_type m))

(cl:ensure-generic-function 'i_val-val :lambda-list '(m))
(cl:defmethod i_val-val ((m <Parameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:i_val-val is deprecated.  Use sandia_hand_msgs-msg:i_val instead.")
  (i_val m))

(cl:ensure-generic-function 'f_val-val :lambda-list '(m))
(cl:defmethod f_val-val ((m <Parameter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:f_val-val is deprecated.  Use sandia_hand_msgs-msg:f_val instead.")
  (f_val m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Parameter>)))
    "Constants for message type '<Parameter>"
  '((:INTEGER . 1)
    (:FLOAT . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Parameter)))
    "Constants for message type 'Parameter"
  '((:INTEGER . 1)
    (:FLOAT . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Parameter>) ostream)
  "Serializes a message object of type '<Parameter>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'i_val)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'i_val)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'i_val)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'i_val)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'f_val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Parameter>) istream)
  "Deserializes a message object of type '<Parameter>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'val_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'i_val)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'i_val)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'i_val)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'i_val)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f_val) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Parameter>)))
  "Returns string type for a message object of type '<Parameter>"
  "sandia_hand_msgs/Parameter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Parameter)))
  "Returns string type for a message object of type 'Parameter"
  "sandia_hand_msgs/Parameter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Parameter>)))
  "Returns md5sum for a message object of type '<Parameter>"
  "57415258934843cb3facecb3fd658b64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Parameter)))
  "Returns md5sum for a message object of type 'Parameter"
  "57415258934843cb3facecb3fd658b64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Parameter>)))
  "Returns full string definition for message of type '<Parameter>"
  (cl:format cl:nil "string  name~%byte    INTEGER=1~%byte    FLOAT=2~%byte    val_type~%uint32  i_val~%float32 f_val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Parameter)))
  "Returns full string definition for message of type 'Parameter"
  (cl:format cl:nil "string  name~%byte    INTEGER=1~%byte    FLOAT=2~%byte    val_type~%uint32  i_val~%float32 f_val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Parameter>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Parameter>))
  "Converts a ROS message object to a list"
  (cl:list 'Parameter
    (cl:cons ':name (name msg))
    (cl:cons ':val_type (val_type msg))
    (cl:cons ':i_val (i_val msg))
    (cl:cons ':f_val (f_val msg))
))
