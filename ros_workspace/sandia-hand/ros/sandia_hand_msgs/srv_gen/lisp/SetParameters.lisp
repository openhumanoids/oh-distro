; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-srv)


;//! \htmlinclude SetParameters-request.msg.html

(cl:defclass <SetParameters-request> (roslisp-msg-protocol:ros-message)
  ((parameters
    :reader parameters
    :initarg :parameters
    :type (cl:vector sandia_hand_msgs-msg:Parameter)
   :initform (cl:make-array 0 :element-type 'sandia_hand_msgs-msg:Parameter :initial-element (cl:make-instance 'sandia_hand_msgs-msg:Parameter))))
)

(cl:defclass SetParameters-request (<SetParameters-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetParameters-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetParameters-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SetParameters-request> is deprecated: use sandia_hand_msgs-srv:SetParameters-request instead.")))

(cl:ensure-generic-function 'parameters-val :lambda-list '(m))
(cl:defmethod parameters-val ((m <SetParameters-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-srv:parameters-val is deprecated.  Use sandia_hand_msgs-srv:parameters instead.")
  (parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetParameters-request>) ostream)
  "Serializes a message object of type '<SetParameters-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'parameters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'parameters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetParameters-request>) istream)
  "Deserializes a message object of type '<SetParameters-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetParameters-request>)))
  "Returns string type for a service object of type '<SetParameters-request>"
  "sandia_hand_msgs/SetParametersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetParameters-request)))
  "Returns string type for a service object of type 'SetParameters-request"
  "sandia_hand_msgs/SetParametersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetParameters-request>)))
  "Returns md5sum for a message object of type '<SetParameters-request>"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetParameters-request)))
  "Returns md5sum for a message object of type 'SetParameters-request"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetParameters-request>)))
  "Returns full string definition for message of type '<SetParameters-request>"
  (cl:format cl:nil "Parameter[] parameters~%~%================================================================================~%MSG: sandia_hand_msgs/Parameter~%string  name~%byte    INTEGER=1~%byte    FLOAT=2~%byte    val_type~%uint32  i_val~%float32 f_val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetParameters-request)))
  "Returns full string definition for message of type 'SetParameters-request"
  (cl:format cl:nil "Parameter[] parameters~%~%================================================================================~%MSG: sandia_hand_msgs/Parameter~%string  name~%byte    INTEGER=1~%byte    FLOAT=2~%byte    val_type~%uint32  i_val~%float32 f_val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetParameters-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'parameters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetParameters-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetParameters-request
    (cl:cons ':parameters (parameters msg))
))
;//! \htmlinclude SetParameters-response.msg.html

(cl:defclass <SetParameters-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetParameters-response (<SetParameters-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetParameters-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetParameters-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SetParameters-response> is deprecated: use sandia_hand_msgs-srv:SetParameters-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetParameters-response>) ostream)
  "Serializes a message object of type '<SetParameters-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetParameters-response>) istream)
  "Deserializes a message object of type '<SetParameters-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetParameters-response>)))
  "Returns string type for a service object of type '<SetParameters-response>"
  "sandia_hand_msgs/SetParametersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetParameters-response)))
  "Returns string type for a service object of type 'SetParameters-response"
  "sandia_hand_msgs/SetParametersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetParameters-response>)))
  "Returns md5sum for a message object of type '<SetParameters-response>"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetParameters-response)))
  "Returns md5sum for a message object of type 'SetParameters-response"
  "3172d4a360e8436e86e2c2ce94e05157")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetParameters-response>)))
  "Returns full string definition for message of type '<SetParameters-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetParameters-response)))
  "Returns full string definition for message of type 'SetParameters-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetParameters-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetParameters-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetParameters-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetParameters)))
  'SetParameters-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetParameters)))
  'SetParameters-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetParameters)))
  "Returns string type for a service object of type '<SetParameters>"
  "sandia_hand_msgs/SetParameters")