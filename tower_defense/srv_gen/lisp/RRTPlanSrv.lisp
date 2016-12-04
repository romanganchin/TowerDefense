; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude RRTPlanSrv-request.msg.html

(cl:defclass <RRTPlanSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RRTPlanSrv-request (<RRTPlanSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RRTPlanSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RRTPlanSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<RRTPlanSrv-request> is deprecated: use tower_defense-srv:RRTPlanSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RRTPlanSrv-request>) ostream)
  "Serializes a message object of type '<RRTPlanSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RRTPlanSrv-request>) istream)
  "Deserializes a message object of type '<RRTPlanSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RRTPlanSrv-request>)))
  "Returns string type for a service object of type '<RRTPlanSrv-request>"
  "tower_defense/RRTPlanSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RRTPlanSrv-request)))
  "Returns string type for a service object of type 'RRTPlanSrv-request"
  "tower_defense/RRTPlanSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RRTPlanSrv-request>)))
  "Returns md5sum for a message object of type '<RRTPlanSrv-request>"
  "b58b29f4d3d5430fc9d5efc2f5262786")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RRTPlanSrv-request)))
  "Returns md5sum for a message object of type 'RRTPlanSrv-request"
  "b58b29f4d3d5430fc9d5efc2f5262786")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RRTPlanSrv-request>)))
  "Returns full string definition for message of type '<RRTPlanSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RRTPlanSrv-request)))
  "Returns full string definition for message of type 'RRTPlanSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RRTPlanSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RRTPlanSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RRTPlanSrv-request
))
;//! \htmlinclude RRTPlanSrv-response.msg.html

(cl:defclass <RRTPlanSrv-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass RRTPlanSrv-response (<RRTPlanSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RRTPlanSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RRTPlanSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<RRTPlanSrv-response> is deprecated: use tower_defense-srv:RRTPlanSrv-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <RRTPlanSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:path-val is deprecated.  Use tower_defense-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RRTPlanSrv-response>) ostream)
  "Serializes a message object of type '<RRTPlanSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RRTPlanSrv-response>) istream)
  "Deserializes a message object of type '<RRTPlanSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RRTPlanSrv-response>)))
  "Returns string type for a service object of type '<RRTPlanSrv-response>"
  "tower_defense/RRTPlanSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RRTPlanSrv-response)))
  "Returns string type for a service object of type 'RRTPlanSrv-response"
  "tower_defense/RRTPlanSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RRTPlanSrv-response>)))
  "Returns md5sum for a message object of type '<RRTPlanSrv-response>"
  "b58b29f4d3d5430fc9d5efc2f5262786")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RRTPlanSrv-response)))
  "Returns md5sum for a message object of type 'RRTPlanSrv-response"
  "b58b29f4d3d5430fc9d5efc2f5262786")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RRTPlanSrv-response>)))
  "Returns full string definition for message of type '<RRTPlanSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point[] path~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RRTPlanSrv-response)))
  "Returns full string definition for message of type 'RRTPlanSrv-response"
  (cl:format cl:nil "geometry_msgs/Point[] path~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RRTPlanSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RRTPlanSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RRTPlanSrv-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RRTPlanSrv)))
  'RRTPlanSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RRTPlanSrv)))
  'RRTPlanSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RRTPlanSrv)))
  "Returns string type for a service object of type '<RRTPlanSrv>"
  "tower_defense/RRTPlanSrv")