; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude BuildRRTSrv-request.msg.html

(cl:defclass <BuildRRTSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BuildRRTSrv-request (<BuildRRTSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BuildRRTSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BuildRRTSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<BuildRRTSrv-request> is deprecated: use tower_defense-srv:BuildRRTSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BuildRRTSrv-request>) ostream)
  "Serializes a message object of type '<BuildRRTSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BuildRRTSrv-request>) istream)
  "Deserializes a message object of type '<BuildRRTSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BuildRRTSrv-request>)))
  "Returns string type for a service object of type '<BuildRRTSrv-request>"
  "tower_defense/BuildRRTSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BuildRRTSrv-request)))
  "Returns string type for a service object of type 'BuildRRTSrv-request"
  "tower_defense/BuildRRTSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BuildRRTSrv-request>)))
  "Returns md5sum for a message object of type '<BuildRRTSrv-request>"
  "9680cc06ea3c47eb5a92d56793715529")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BuildRRTSrv-request)))
  "Returns md5sum for a message object of type 'BuildRRTSrv-request"
  "9680cc06ea3c47eb5a92d56793715529")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BuildRRTSrv-request>)))
  "Returns full string definition for message of type '<BuildRRTSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BuildRRTSrv-request)))
  "Returns full string definition for message of type 'BuildRRTSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BuildRRTSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BuildRRTSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BuildRRTSrv-request
))
;//! \htmlinclude BuildRRTSrv-response.msg.html

(cl:defclass <BuildRRTSrv-response> (roslisp-msg-protocol:ros-message)
  ((rrt
    :reader rrt
    :initarg :rrt
    :type (cl:vector tower_defense-msg:RRTNode)
   :initform (cl:make-array 0 :element-type 'tower_defense-msg:RRTNode :initial-element (cl:make-instance 'tower_defense-msg:RRTNode))))
)

(cl:defclass BuildRRTSrv-response (<BuildRRTSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BuildRRTSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BuildRRTSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<BuildRRTSrv-response> is deprecated: use tower_defense-srv:BuildRRTSrv-response instead.")))

(cl:ensure-generic-function 'rrt-val :lambda-list '(m))
(cl:defmethod rrt-val ((m <BuildRRTSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:rrt-val is deprecated.  Use tower_defense-srv:rrt instead.")
  (rrt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BuildRRTSrv-response>) ostream)
  "Serializes a message object of type '<BuildRRTSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rrt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rrt))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BuildRRTSrv-response>) istream)
  "Deserializes a message object of type '<BuildRRTSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rrt) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rrt)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tower_defense-msg:RRTNode))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BuildRRTSrv-response>)))
  "Returns string type for a service object of type '<BuildRRTSrv-response>"
  "tower_defense/BuildRRTSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BuildRRTSrv-response)))
  "Returns string type for a service object of type 'BuildRRTSrv-response"
  "tower_defense/BuildRRTSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BuildRRTSrv-response>)))
  "Returns md5sum for a message object of type '<BuildRRTSrv-response>"
  "9680cc06ea3c47eb5a92d56793715529")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BuildRRTSrv-response)))
  "Returns md5sum for a message object of type 'BuildRRTSrv-response"
  "9680cc06ea3c47eb5a92d56793715529")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BuildRRTSrv-response>)))
  "Returns full string definition for message of type '<BuildRRTSrv-response>"
  (cl:format cl:nil "~%~%RRTNode[] rrt~%~%~%================================================================================~%MSG: tower_defense/RRTNode~%# Location of the tree node.~%geometry_msgs/Point location~%~%# Index of parent node, -1 if there is no parent (root node).~%int32 parent~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BuildRRTSrv-response)))
  "Returns full string definition for message of type 'BuildRRTSrv-response"
  (cl:format cl:nil "~%~%RRTNode[] rrt~%~%~%================================================================================~%MSG: tower_defense/RRTNode~%# Location of the tree node.~%geometry_msgs/Point location~%~%# Index of parent node, -1 if there is no parent (root node).~%int32 parent~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BuildRRTSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rrt) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BuildRRTSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BuildRRTSrv-response
    (cl:cons ':rrt (rrt msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BuildRRTSrv)))
  'BuildRRTSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BuildRRTSrv)))
  'BuildRRTSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BuildRRTSrv)))
  "Returns string type for a service object of type '<BuildRRTSrv>"
  "tower_defense/BuildRRTSrv")