; Auto-generated. Do not edit!


(cl:in-package kinect_anywhere-msg)


;//! \htmlinclude BodyFrame.msg.html

(cl:defclass <BodyFrame> (roslisp-msg-protocol:ros-message)
  ((bodies
    :reader bodies
    :initarg :bodies
    :type (cl:vector kinect_anywhere-msg:Body)
   :initform (cl:make-array 0 :element-type 'kinect_anywhere-msg:Body :initial-element (cl:make-instance 'kinect_anywhere-msg:Body))))
)

(cl:defclass BodyFrame (<BodyFrame>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyFrame>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyFrame)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect_anywhere-msg:<BodyFrame> is deprecated: use kinect_anywhere-msg:BodyFrame instead.")))

(cl:ensure-generic-function 'bodies-val :lambda-list '(m))
(cl:defmethod bodies-val ((m <BodyFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_anywhere-msg:bodies-val is deprecated.  Use kinect_anywhere-msg:bodies instead.")
  (bodies m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyFrame>) ostream)
  "Serializes a message object of type '<BodyFrame>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bodies))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bodies))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyFrame>) istream)
  "Deserializes a message object of type '<BodyFrame>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bodies) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bodies)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kinect_anywhere-msg:Body))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyFrame>)))
  "Returns string type for a message object of type '<BodyFrame>"
  "kinect_anywhere/BodyFrame")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyFrame)))
  "Returns string type for a message object of type 'BodyFrame"
  "kinect_anywhere/BodyFrame")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyFrame>)))
  "Returns md5sum for a message object of type '<BodyFrame>"
  "c8ca6cd86f661b94d3cea79f1dad0380")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyFrame)))
  "Returns md5sum for a message object of type 'BodyFrame"
  "c8ca6cd86f661b94d3cea79f1dad0380")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyFrame>)))
  "Returns full string definition for message of type '<BodyFrame>"
  (cl:format cl:nil "Body[] bodies~%~%================================================================================~%MSG: kinect_anywhere/Body~%Header header~%uint64 trackingId~%JointPositionAndState[] jointPositions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: kinect_anywhere/JointPositionAndState~%int32 trackingState~%geometry_msgs/Point position~%int32 jointType~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyFrame)))
  "Returns full string definition for message of type 'BodyFrame"
  (cl:format cl:nil "Body[] bodies~%~%================================================================================~%MSG: kinect_anywhere/Body~%Header header~%uint64 trackingId~%JointPositionAndState[] jointPositions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: kinect_anywhere/JointPositionAndState~%int32 trackingState~%geometry_msgs/Point position~%int32 jointType~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyFrame>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bodies) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyFrame>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyFrame
    (cl:cons ':bodies (bodies msg))
))
