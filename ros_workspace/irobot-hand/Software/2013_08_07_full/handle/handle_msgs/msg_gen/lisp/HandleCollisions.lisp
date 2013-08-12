; Auto-generated. Do not edit!


(cl:in-package handle_msgs-msg)


;//! \htmlinclude HandleCollisions.msg.html

(cl:defclass <HandleCollisions> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (collisions
    :reader collisions
    :initarg :collisions
    :type (cl:vector handle_msgs-msg:Collision)
   :initform (cl:make-array 0 :element-type 'handle_msgs-msg:Collision :initial-element (cl:make-instance 'handle_msgs-msg:Collision))))
)

(cl:defclass HandleCollisions (<HandleCollisions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandleCollisions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandleCollisions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name handle_msgs-msg:<HandleCollisions> is deprecated: use handle_msgs-msg:HandleCollisions instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HandleCollisions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:header-val is deprecated.  Use handle_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'collisions-val :lambda-list '(m))
(cl:defmethod collisions-val ((m <HandleCollisions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader handle_msgs-msg:collisions-val is deprecated.  Use handle_msgs-msg:collisions instead.")
  (collisions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandleCollisions>) ostream)
  "Serializes a message object of type '<HandleCollisions>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'collisions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'collisions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandleCollisions>) istream)
  "Deserializes a message object of type '<HandleCollisions>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'collisions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'collisions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'handle_msgs-msg:Collision))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandleCollisions>)))
  "Returns string type for a message object of type '<HandleCollisions>"
  "handle_msgs/HandleCollisions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandleCollisions)))
  "Returns string type for a message object of type 'HandleCollisions"
  "handle_msgs/HandleCollisions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandleCollisions>)))
  "Returns md5sum for a message object of type '<HandleCollisions>"
  "4beb2df6b938c5819449df17bcfbc49b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandleCollisions)))
  "Returns md5sum for a message object of type 'HandleCollisions"
  "4beb2df6b938c5819449df17bcfbc49b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandleCollisions>)))
  "Returns full string definition for message of type '<HandleCollisions>"
  (cl:format cl:nil "# This is sensors of the HANDLE hand after calibration and data manipulation~%# published from the package sensors, by the sensors_publisher~%~%# not all the sensors were included, but only the one which were addressed at the moment~%~%# Currently only used for time stamp.  ~%Header header~%~%Collision[] collisions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: handle_msgs/Collision~%# This is basic collision message~%# it is used in HandleCollisions to build an array~%~%string frame_id~%# finger[0]/proximal_link~%# finger[0]/distal_link~%# finger[1]/proximal_link~%# finger[1]/distal_link~%# finger[2]/proximal_link~%# finger[2]/distal_link~%# base_link~%~%int32 sensor_id~%# index of sensor~%~%float32 intensity~%~%# location of sensor on the surface of the finger in the link frame~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandleCollisions)))
  "Returns full string definition for message of type 'HandleCollisions"
  (cl:format cl:nil "# This is sensors of the HANDLE hand after calibration and data manipulation~%# published from the package sensors, by the sensors_publisher~%~%# not all the sensors were included, but only the one which were addressed at the moment~%~%# Currently only used for time stamp.  ~%Header header~%~%Collision[] collisions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: handle_msgs/Collision~%# This is basic collision message~%# it is used in HandleCollisions to build an array~%~%string frame_id~%# finger[0]/proximal_link~%# finger[0]/distal_link~%# finger[1]/proximal_link~%# finger[1]/distal_link~%# finger[2]/proximal_link~%# finger[2]/distal_link~%# base_link~%~%int32 sensor_id~%# index of sensor~%~%float32 intensity~%~%# location of sensor on the surface of the finger in the link frame~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandleCollisions>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'collisions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandleCollisions>))
  "Converts a ROS message object to a list"
  (cl:list 'HandleCollisions
    (cl:cons ':header (header msg))
    (cl:cons ':collisions (collisions msg))
))
