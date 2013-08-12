; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-msg)


;//! \htmlinclude SimpleGrasp.msg.html

(cl:defclass <SimpleGrasp> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (closed_amount
    :reader closed_amount
    :initarg :closed_amount
    :type cl:float
    :initform 0.0))
)

(cl:defclass SimpleGrasp (<SimpleGrasp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimpleGrasp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimpleGrasp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-msg:<SimpleGrasp> is deprecated: use sandia_hand_msgs-msg:SimpleGrasp instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SimpleGrasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:name-val is deprecated.  Use sandia_hand_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'closed_amount-val :lambda-list '(m))
(cl:defmethod closed_amount-val ((m <SimpleGrasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-msg:closed_amount-val is deprecated.  Use sandia_hand_msgs-msg:closed_amount instead.")
  (closed_amount m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimpleGrasp>) ostream)
  "Serializes a message object of type '<SimpleGrasp>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'closed_amount))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimpleGrasp>) istream)
  "Deserializes a message object of type '<SimpleGrasp>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'closed_amount) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimpleGrasp>)))
  "Returns string type for a message object of type '<SimpleGrasp>"
  "sandia_hand_msgs/SimpleGrasp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleGrasp)))
  "Returns string type for a message object of type 'SimpleGrasp"
  "sandia_hand_msgs/SimpleGrasp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimpleGrasp>)))
  "Returns md5sum for a message object of type '<SimpleGrasp>"
  "95873588e5f0e32a373b23f95e571087")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimpleGrasp)))
  "Returns md5sum for a message object of type 'SimpleGrasp"
  "95873588e5f0e32a373b23f95e571087")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimpleGrasp>)))
  "Returns full string definition for message of type '<SimpleGrasp>"
  (cl:format cl:nil "string  name~%float64 closed_amount  ~%# closed_amount = 0 means fully open~%# closed_amount = 1 means fully closed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimpleGrasp)))
  "Returns full string definition for message of type 'SimpleGrasp"
  (cl:format cl:nil "string  name~%float64 closed_amount  ~%# closed_amount = 0 means fully open~%# closed_amount = 1 means fully closed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimpleGrasp>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimpleGrasp>))
  "Converts a ROS message object to a list"
  (cl:list 'SimpleGrasp
    (cl:cons ':name (name msg))
    (cl:cons ':closed_amount (closed_amount msg))
))
