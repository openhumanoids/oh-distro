; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-srv)


;//! \htmlinclude GetParameters-request.msg.html

(cl:defclass <GetParameters-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetParameters-request (<GetParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<GetParameters-request> is deprecated: use sandia_hand_msgs-srv:GetParameters-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParameters-request>) ostream)
  "Serializes a message object of type '<GetParameters-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParameters-request>) istream)
  "Deserializes a message object of type '<GetParameters-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParameters-request>)))
  "Returns string type for a service object of type '<GetParameters-request>"
  "sandia_hand_msgs/GetParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParameters-request)))
  "Returns string type for a service object of type 'GetParameters-request"
  "sandia_hand_msgs/GetParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParameters-request>)))
  "Returns md5sum for a message object of type '<GetParameters-request>"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParameters-request)))
  "Returns md5sum for a message object of type 'GetParameters-request"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParameters-request>)))
  "Returns full string definition for message of type '<GetParameters-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParameters-request)))
  "Returns full string definition for message of type 'GetParameters-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParameters-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParameters-request
))
;//! \htmlinclude GetParameters-response.msg.html

(cl:defclass <GetParameters-response> (roslisp-msg-protocol:ros-message)
  ((parameters
    :reader parameters
    :initarg :parameters
    :type (cl:vector sandia_hand_msgs-msg:Parameter)
   :initform (cl:make-array 0 :element-type 'sandia_hand_msgs-msg:Parameter :initial-element (cl:make-instance 'sandia_hand_msgs-msg:Parameter))))
)

(cl:defclass GetParameters-response (<GetParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<GetParameters-response> is deprecated: use sandia_hand_msgs-srv:GetParameters-response instead.")))

(cl:ensure-generic-function 'parameters-val :lambda-list '(m))
(cl:defmethod parameters-val ((m <GetParameters-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-srv:parameters-val is deprecated.  Use sandia_hand_msgs-srv:parameters instead.")
  (parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParameters-response>) ostream)
  "Serializes a message object of type '<GetParameters-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'parameters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'parameters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParameters-response>) istream)
  "Deserializes a message object of type '<GetParameters-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'parameters) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'parameters)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sandia_hand_msgs-msg:Parameter))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParameters-response>)))
  "Returns string type for a service object of type '<GetParameters-response>"
  "sandia_hand_msgs/GetParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParameters-response)))
  "Returns string type for a service object of type 'GetParameters-response"
  "sandia_hand_msgs/GetParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParameters-response>)))
  "Returns md5sum for a message object of type '<GetParameters-response>"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParameters-response)))
  "Returns md5sum for a message object of type 'GetParameters-response"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParameters-response>)))
  "Returns full string definition for message of type '<GetParameters-response>"
  (cl:format cl:nil "Parameter[] parameters~%~%~%================================================================================~%MSG: sandia_hand_msgs/Parameter~%string  name~%byte    INTEGER=1~%byte    FLOAT=2~%byte    val_type~%uint32  i_val~%float32 f_val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParameters-response)))
  "Returns full string definition for message of type 'GetParameters-response"
  (cl:format cl:nil "Parameter[] parameters~%~%~%================================================================================~%MSG: sandia_hand_msgs/Parameter~%string  name~%byte    INTEGER=1~%byte    FLOAT=2~%byte    val_type~%uint32  i_val~%float32 f_val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParameters-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'parameters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParameters-response
    (cl:cons ':parameters (parameters msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetParameters)))
  'GetParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetParameters)))
  'GetParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParameters)))
  "Returns string type for a service object of type '<GetParameters>"
  "sandia_hand_msgs/GetParameters")