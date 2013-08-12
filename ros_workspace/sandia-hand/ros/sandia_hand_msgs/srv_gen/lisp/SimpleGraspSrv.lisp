; Auto-generated. Do not edit!


(cl:in-package sandia_hand_msgs-srv)


;//! \htmlinclude SimpleGraspSrv-request.msg.html

(cl:defclass <SimpleGraspSrv-request> (roslisp-msg-protocol:ros-message)
  ((grasp
    :reader grasp
    :initarg :grasp
    :type sandia_hand_msgs-msg:SimpleGrasp
    :initform (cl:make-instance 'sandia_hand_msgs-msg:SimpleGrasp)))
)

(cl:defclass SimpleGraspSrv-request (<SimpleGraspSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimpleGraspSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimpleGraspSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SimpleGraspSrv-request> is deprecated: use sandia_hand_msgs-srv:SimpleGraspSrv-request instead.")))

(cl:ensure-generic-function 'grasp-val :lambda-list '(m))
(cl:defmethod grasp-val ((m <SimpleGraspSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sandia_hand_msgs-srv:grasp-val is deprecated.  Use sandia_hand_msgs-srv:grasp instead.")
  (grasp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimpleGraspSrv-request>) ostream)
  "Serializes a message object of type '<SimpleGraspSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grasp) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimpleGraspSrv-request>) istream)
  "Deserializes a message object of type '<SimpleGraspSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grasp) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimpleGraspSrv-request>)))
  "Returns string type for a service object of type '<SimpleGraspSrv-request>"
  "sandia_hand_msgs/SimpleGraspSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleGraspSrv-request)))
  "Returns string type for a service object of type 'SimpleGraspSrv-request"
  "sandia_hand_msgs/SimpleGraspSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimpleGraspSrv-request>)))
  "Returns md5sum for a message object of type '<SimpleGraspSrv-request>"
  "271a4db012a7d00f6c0fed086d30155c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimpleGraspSrv-request)))
  "Returns md5sum for a message object of type 'SimpleGraspSrv-request"
  "271a4db012a7d00f6c0fed086d30155c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimpleGraspSrv-request>)))
  "Returns full string definition for message of type '<SimpleGraspSrv-request>"
  (cl:format cl:nil "SimpleGrasp grasp~%~%================================================================================~%MSG: sandia_hand_msgs/SimpleGrasp~%string  name~%float64 closed_amount  ~%# closed_amount = 0 means fully open~%# closed_amount = 1 means fully closed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimpleGraspSrv-request)))
  "Returns full string definition for message of type 'SimpleGraspSrv-request"
  (cl:format cl:nil "SimpleGrasp grasp~%~%================================================================================~%MSG: sandia_hand_msgs/SimpleGrasp~%string  name~%float64 closed_amount  ~%# closed_amount = 0 means fully open~%# closed_amount = 1 means fully closed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimpleGraspSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grasp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimpleGraspSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SimpleGraspSrv-request
    (cl:cons ':grasp (grasp msg))
))
;//! \htmlinclude SimpleGraspSrv-response.msg.html

(cl:defclass <SimpleGraspSrv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SimpleGraspSrv-response (<SimpleGraspSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimpleGraspSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimpleGraspSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sandia_hand_msgs-srv:<SimpleGraspSrv-response> is deprecated: use sandia_hand_msgs-srv:SimpleGraspSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimpleGraspSrv-response>) ostream)
  "Serializes a message object of type '<SimpleGraspSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimpleGraspSrv-response>) istream)
  "Deserializes a message object of type '<SimpleGraspSrv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimpleGraspSrv-response>)))
  "Returns string type for a service object of type '<SimpleGraspSrv-response>"
  "sandia_hand_msgs/SimpleGraspSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleGraspSrv-response)))
  "Returns string type for a service object of type 'SimpleGraspSrv-response"
  "sandia_hand_msgs/SimpleGraspSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimpleGraspSrv-response>)))
  "Returns md5sum for a message object of type '<SimpleGraspSrv-response>"
  "271a4db012a7d00f6c0fed086d30155c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimpleGraspSrv-response)))
  "Returns md5sum for a message object of type 'SimpleGraspSrv-response"
  "271a4db012a7d00f6c0fed086d30155c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimpleGraspSrv-response>)))
  "Returns full string definition for message of type '<SimpleGraspSrv-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimpleGraspSrv-response)))
  "Returns full string definition for message of type 'SimpleGraspSrv-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimpleGraspSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimpleGraspSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SimpleGraspSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SimpleGraspSrv)))
  'SimpleGraspSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SimpleGraspSrv)))
  'SimpleGraspSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleGraspSrv)))
  "Returns string type for a service object of type '<SimpleGraspSrv>"
  "sandia_hand_msgs/SimpleGraspSrv")