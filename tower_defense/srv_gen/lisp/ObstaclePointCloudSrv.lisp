; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude ObstaclePointCloudSrv-request.msg.html

(cl:defclass <ObstaclePointCloudSrv-request> (roslisp-msg-protocol:ros-message)
  ((R
    :reader R
    :initarg :R
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (T
    :reader T
    :initarg :T
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (P
    :reader P
    :initarg :P
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass ObstaclePointCloudSrv-request (<ObstaclePointCloudSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstaclePointCloudSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstaclePointCloudSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<ObstaclePointCloudSrv-request> is deprecated: use tower_defense-srv:ObstaclePointCloudSrv-request instead.")))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <ObstaclePointCloudSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:R-val is deprecated.  Use tower_defense-srv:R instead.")
  (R m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <ObstaclePointCloudSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:T-val is deprecated.  Use tower_defense-srv:T instead.")
  (T m))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <ObstaclePointCloudSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:P-val is deprecated.  Use tower_defense-srv:P instead.")
  (P m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstaclePointCloudSrv-request>) ostream)
  "Serializes a message object of type '<ObstaclePointCloudSrv-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'R))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'T) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'P))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstaclePointCloudSrv-request>) istream)
  "Deserializes a message object of type '<ObstaclePointCloudSrv-request>"
  (cl:setf (cl:slot-value msg 'R) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'R)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'T) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'P) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'P)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstaclePointCloudSrv-request>)))
  "Returns string type for a service object of type '<ObstaclePointCloudSrv-request>"
  "tower_defense/ObstaclePointCloudSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstaclePointCloudSrv-request)))
  "Returns string type for a service object of type 'ObstaclePointCloudSrv-request"
  "tower_defense/ObstaclePointCloudSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstaclePointCloudSrv-request>)))
  "Returns md5sum for a message object of type '<ObstaclePointCloudSrv-request>"
  "223e64a99fafffe6bb9729d31c47a42e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstaclePointCloudSrv-request)))
  "Returns md5sum for a message object of type 'ObstaclePointCloudSrv-request"
  "223e64a99fafffe6bb9729d31c47a42e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstaclePointCloudSrv-request>)))
  "Returns full string definition for message of type '<ObstaclePointCloudSrv-request>"
  (cl:format cl:nil "float32[9] R~%geometry_msgs/Point32 T~%geometry_msgs/Point32[] P~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstaclePointCloudSrv-request)))
  "Returns full string definition for message of type 'ObstaclePointCloudSrv-request"
  (cl:format cl:nil "float32[9] R~%geometry_msgs/Point32 T~%geometry_msgs/Point32[] P~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstaclePointCloudSrv-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'R) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'T))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstaclePointCloudSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstaclePointCloudSrv-request
    (cl:cons ':R (R msg))
    (cl:cons ':T (T msg))
    (cl:cons ':P (P msg))
))
;//! \htmlinclude ObstaclePointCloudSrv-response.msg.html

(cl:defclass <ObstaclePointCloudSrv-response> (roslisp-msg-protocol:ros-message)
  ((P_prime
    :reader P_prime
    :initarg :P_prime
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass ObstaclePointCloudSrv-response (<ObstaclePointCloudSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstaclePointCloudSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstaclePointCloudSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<ObstaclePointCloudSrv-response> is deprecated: use tower_defense-srv:ObstaclePointCloudSrv-response instead.")))

(cl:ensure-generic-function 'P_prime-val :lambda-list '(m))
(cl:defmethod P_prime-val ((m <ObstaclePointCloudSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:P_prime-val is deprecated.  Use tower_defense-srv:P_prime instead.")
  (P_prime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstaclePointCloudSrv-response>) ostream)
  "Serializes a message object of type '<ObstaclePointCloudSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P_prime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'P_prime))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstaclePointCloudSrv-response>) istream)
  "Deserializes a message object of type '<ObstaclePointCloudSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'P_prime) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'P_prime)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstaclePointCloudSrv-response>)))
  "Returns string type for a service object of type '<ObstaclePointCloudSrv-response>"
  "tower_defense/ObstaclePointCloudSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstaclePointCloudSrv-response)))
  "Returns string type for a service object of type 'ObstaclePointCloudSrv-response"
  "tower_defense/ObstaclePointCloudSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstaclePointCloudSrv-response>)))
  "Returns md5sum for a message object of type '<ObstaclePointCloudSrv-response>"
  "223e64a99fafffe6bb9729d31c47a42e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstaclePointCloudSrv-response)))
  "Returns md5sum for a message object of type 'ObstaclePointCloudSrv-response"
  "223e64a99fafffe6bb9729d31c47a42e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstaclePointCloudSrv-response>)))
  "Returns full string definition for message of type '<ObstaclePointCloudSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32[] P_prime~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstaclePointCloudSrv-response)))
  "Returns full string definition for message of type 'ObstaclePointCloudSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32[] P_prime~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstaclePointCloudSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P_prime) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstaclePointCloudSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstaclePointCloudSrv-response
    (cl:cons ':P_prime (P_prime msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ObstaclePointCloudSrv)))
  'ObstaclePointCloudSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ObstaclePointCloudSrv)))
  'ObstaclePointCloudSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstaclePointCloudSrv)))
  "Returns string type for a service object of type '<ObstaclePointCloudSrv>"
  "tower_defense/ObstaclePointCloudSrv")