; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-srv)


;//! \htmlinclude SetFingerHome-request.msg.html

(cl:defclass <SetFingerHome-request> (roslisp-msg-protocol:ros-message)
  ((finger_idx
    :reader finger_idx
    :initarg :finger_idx
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetFingerHome-request (<SetFingerHome-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFingerHome-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFingerHome-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SetFingerHome-request> is deprecated: use sandia_hand_msgs-srv:SetFingerHome-request instead.")))

(cl:ensure-generic-function 'finger_idx-val :lambda-list '(m))
(cl:defmethod finger_idx-val ((m <SetFingerHome-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-srv:finger_idx-val is deprecated.  Use sandia_hand_msgs-srv:finger_idx instead.")
  (finger_idx m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFingerHome-request>) ostream)
  "Serializes a message object of type '<SetFingerHome-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger_idx)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFingerHome-request>) istream)
  "Deserializes a message object of type '<SetFingerHome-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger_idx)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFingerHome-request>)))
  "Returns string type for a service object of type '<SetFingerHome-request>"
  "sandia_hand_msgs/SetFingerHomeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFingerHome-request)))
  "Returns string type for a service object of type 'SetFingerHome-request"
  "sandia_hand_msgs/SetFingerHomeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFingerHome-request>)))
  "Returns md5sum for a message object of type '<SetFingerHome-request>"
  "a0a762987711da44062453b8336a93ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFingerHome-request)))
  "Returns md5sum for a message object of type 'SetFingerHome-request"
  "a0a762987711da44062453b8336a93ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFingerHome-request>)))
  "Returns full string definition for message of type '<SetFingerHome-request>"
  (cl:format cl:nil "uint8 finger_idx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFingerHome-request)))
  "Returns full string definition for message of type 'SetFingerHome-request"
  (cl:format cl:nil "uint8 finger_idx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFingerHome-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFingerHome-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFingerHome-request
    (cl:cons ':finger_idx (finger_idx msg))
))
;//! \htmlinclude SetFingerHome-response.msg.html

(cl:defclass <SetFingerHome-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetFingerHome-response (<SetFingerHome-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFingerHome-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFingerHome-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SetFingerHome-response> is deprecated: use sandia_hand_msgs-srv:SetFingerHome-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFingerHome-response>) ostream)
  "Serializes a message object of type '<SetFingerHome-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFingerHome-response>) istream)
  "Deserializes a message object of type '<SetFingerHome-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFingerHome-response>)))
  "Returns string type for a service object of type '<SetFingerHome-response>"
  "sandia_hand_msgs/SetFingerHomeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFingerHome-response)))
  "Returns string type for a service object of type 'SetFingerHome-response"
  "sandia_hand_msgs/SetFingerHomeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFingerHome-response>)))
  "Returns md5sum for a message object of type '<SetFingerHome-response>"
  "a0a762987711da44062453b8336a93ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFingerHome-response)))
  "Returns md5sum for a message object of type 'SetFingerHome-response"
  "a0a762987711da44062453b8336a93ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFingerHome-response>)))
  "Returns full string definition for message of type '<SetFingerHome-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFingerHome-response)))
  "Returns full string definition for message of type 'SetFingerHome-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFingerHome-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFingerHome-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFingerHome-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFingerHome)))
  'SetFingerHome-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFingerHome)))
  'SetFingerHome-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFingerHome)))
  "Returns string type for a service object of type '<SetFingerHome>"
  "sandia_hand_msgs/SetFingerHome")