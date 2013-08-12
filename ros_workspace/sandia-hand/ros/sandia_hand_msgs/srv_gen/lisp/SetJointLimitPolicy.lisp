; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-srv)


;//! \htmlinclude SetJointLimitPolicy-request.msg.html

(cl:defclass <SetJointLimitPolicy-request> (roslisp-msg-protocol:ros-message)
  ((policy
    :reader policy
    :initarg :policy
    :type cl:string
    :initform ""))
)

(cl:defclass SetJointLimitPolicy-request (<SetJointLimitPolicy-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointLimitPolicy-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointLimitPolicy-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SetJointLimitPolicy-request> is deprecated: use sandia_hand_msgs-srv:SetJointLimitPolicy-request instead.")))

(cl:ensure-generic-function 'policy-val :lambda-list '(m))
(cl:defmethod policy-val ((m <SetJointLimitPolicy-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-srv:policy-val is deprecated.  Use sandia_hand_msgs-srv:policy instead.")
  (policy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointLimitPolicy-request>) ostream)
  "Serializes a message object of type '<SetJointLimitPolicy-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'policy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'policy))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointLimitPolicy-request>) istream)
  "Deserializes a message object of type '<SetJointLimitPolicy-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'policy) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'policy) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointLimitPolicy-request>)))
  "Returns string type for a service object of type '<SetJointLimitPolicy-request>"
  "sandia_hand_msgs/SetJointLimitPolicyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointLimitPolicy-request)))
  "Returns string type for a service object of type 'SetJointLimitPolicy-request"
  "sandia_hand_msgs/SetJointLimitPolicyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointLimitPolicy-request>)))
  "Returns md5sum for a message object of type '<SetJointLimitPolicy-request>"
  "e910d3ed1e2e8ad67a9ceacdc38b1c45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointLimitPolicy-request)))
  "Returns md5sum for a message object of type 'SetJointLimitPolicy-request"
  "e910d3ed1e2e8ad67a9ceacdc38b1c45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointLimitPolicy-request>)))
  "Returns full string definition for message of type '<SetJointLimitPolicy-request>"
  (cl:format cl:nil "string policy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointLimitPolicy-request)))
  "Returns full string definition for message of type 'SetJointLimitPolicy-request"
  (cl:format cl:nil "string policy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointLimitPolicy-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'policy))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointLimitPolicy-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointLimitPolicy-request
    (cl:cons ':policy (policy msg))
))
;//! \htmlinclude SetJointLimitPolicy-response.msg.html

(cl:defclass <SetJointLimitPolicy-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetJointLimitPolicy-response (<SetJointLimitPolicy-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointLimitPolicy-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointLimitPolicy-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SetJointLimitPolicy-response> is deprecated: use sandia_hand_msgs-srv:SetJointLimitPolicy-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointLimitPolicy-response>) ostream)
  "Serializes a message object of type '<SetJointLimitPolicy-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointLimitPolicy-response>) istream)
  "Deserializes a message object of type '<SetJointLimitPolicy-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointLimitPolicy-response>)))
  "Returns string type for a service object of type '<SetJointLimitPolicy-response>"
  "sandia_hand_msgs/SetJointLimitPolicyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointLimitPolicy-response)))
  "Returns string type for a service object of type 'SetJointLimitPolicy-response"
  "sandia_hand_msgs/SetJointLimitPolicyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointLimitPolicy-response>)))
  "Returns md5sum for a message object of type '<SetJointLimitPolicy-response>"
  "e910d3ed1e2e8ad67a9ceacdc38b1c45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointLimitPolicy-response)))
  "Returns md5sum for a message object of type 'SetJointLimitPolicy-response"
  "e910d3ed1e2e8ad67a9ceacdc38b1c45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointLimitPolicy-response>)))
  "Returns full string definition for message of type '<SetJointLimitPolicy-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointLimitPolicy-response)))
  "Returns full string definition for message of type 'SetJointLimitPolicy-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointLimitPolicy-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointLimitPolicy-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointLimitPolicy-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointLimitPolicy)))
  'SetJointLimitPolicy-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointLimitPolicy)))
  'SetJointLimitPolicy-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointLimitPolicy)))
  "Returns string type for a service object of type '<SetJointLimitPolicy>"
  "sandia_hand_msgs/SetJointLimitPolicy")