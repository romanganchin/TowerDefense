; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude MakePathSrv-request.msg.html

(cl:defclass <MakePathSrv-request> (roslisp-msg-protocol:ros-message)
  ((point_cloud
    :reader point_cloud
    :initarg :point_cloud
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass MakePathSrv-request (<MakePathSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MakePathSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MakePathSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<MakePathSrv-request> is deprecated: use tower_defense-srv:MakePathSrv-request instead.")))

(cl:ensure-generic-function 'point_cloud-val :lambda-list '(m))
(cl:defmethod point_cloud-val ((m <MakePathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:point_cloud-val is deprecated.  Use tower_defense-srv:point_cloud instead.")
  (point_cloud m))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <MakePathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:start-val is deprecated.  Use tower_defense-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <MakePathSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:end-val is deprecated.  Use tower_defense-srv:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MakePathSrv-request>) ostream)
  "Serializes a message object of type '<MakePathSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point_cloud))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'point_cloud))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MakePathSrv-request>) istream)
  "Deserializes a message object of type '<MakePathSrv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point_cloud) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point_cloud)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MakePathSrv-request>)))
  "Returns string type for a service object of type '<MakePathSrv-request>"
  "tower_defense/MakePathSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MakePathSrv-request)))
  "Returns string type for a service object of type 'MakePathSrv-request"
  "tower_defense/MakePathSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MakePathSrv-request>)))
  "Returns md5sum for a message object of type '<MakePathSrv-request>"
  "e7e8c90999c49a7956a41be5569d7437")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MakePathSrv-request)))
  "Returns md5sum for a message object of type 'MakePathSrv-request"
  "e7e8c90999c49a7956a41be5569d7437")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MakePathSrv-request>)))
  "Returns full string definition for message of type '<MakePathSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point[] point_cloud~%geometry_msgs/Point start~%geometry_msgs/Point end~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MakePathSrv-request)))
  "Returns full string definition for message of type 'MakePathSrv-request"
  (cl:format cl:nil "geometry_msgs/Point[] point_cloud~%geometry_msgs/Point start~%geometry_msgs/Point end~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MakePathSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point_cloud) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MakePathSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MakePathSrv-request
    (cl:cons ':point_cloud (point_cloud msg))
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
;//! \htmlinclude MakePathSrv-response.msg.html

(cl:defclass <MakePathSrv-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass MakePathSrv-response (<MakePathSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MakePathSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MakePathSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<MakePathSrv-response> is deprecated: use tower_defense-srv:MakePathSrv-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <MakePathSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:path-val is deprecated.  Use tower_defense-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MakePathSrv-response>) ostream)
  "Serializes a message object of type '<MakePathSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MakePathSrv-response>) istream)
  "Deserializes a message object of type '<MakePathSrv-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MakePathSrv-response>)))
  "Returns string type for a service object of type '<MakePathSrv-response>"
  "tower_defense/MakePathSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MakePathSrv-response)))
  "Returns string type for a service object of type 'MakePathSrv-response"
  "tower_defense/MakePathSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MakePathSrv-response>)))
  "Returns md5sum for a message object of type '<MakePathSrv-response>"
  "e7e8c90999c49a7956a41be5569d7437")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MakePathSrv-response)))
  "Returns md5sum for a message object of type 'MakePathSrv-response"
  "e7e8c90999c49a7956a41be5569d7437")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MakePathSrv-response>)))
  "Returns full string definition for message of type '<MakePathSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MakePathSrv-response)))
  "Returns full string definition for message of type 'MakePathSrv-response"
  (cl:format cl:nil "geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MakePathSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MakePathSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MakePathSrv-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MakePathSrv)))
  'MakePathSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MakePathSrv)))
  'MakePathSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MakePathSrv)))
  "Returns string type for a service object of type '<MakePathSrv>"
  "tower_defense/MakePathSrv")