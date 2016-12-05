; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude MoveCreepersSrv-request.msg.html

(cl:defclass <MoveCreepersSrv-request> (roslisp-msg-protocol:ros-message)
  ((create_new
    :reader create_new
    :initarg :create_new
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass MoveCreepersSrv-request (<MoveCreepersSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCreepersSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCreepersSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<MoveCreepersSrv-request> is deprecated: use tower_defense-srv:MoveCreepersSrv-request instead.")))

(cl:ensure-generic-function 'create_new-val :lambda-list '(m))
(cl:defmethod create_new-val ((m <MoveCreepersSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:create_new-val is deprecated.  Use tower_defense-srv:create_new instead.")
  (create_new m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCreepersSrv-request>) ostream)
  "Serializes a message object of type '<MoveCreepersSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'create_new) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCreepersSrv-request>) istream)
  "Deserializes a message object of type '<MoveCreepersSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'create_new) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveCreepersSrv-request>)))
  "Returns string type for a service object of type '<MoveCreepersSrv-request>"
  "tower_defense/MoveCreepersSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCreepersSrv-request)))
  "Returns string type for a service object of type 'MoveCreepersSrv-request"
  "tower_defense/MoveCreepersSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveCreepersSrv-request>)))
  "Returns md5sum for a message object of type '<MoveCreepersSrv-request>"
  "f428434760fd966922080bb0858f16a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCreepersSrv-request)))
  "Returns md5sum for a message object of type 'MoveCreepersSrv-request"
  "f428434760fd966922080bb0858f16a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCreepersSrv-request>)))
  "Returns full string definition for message of type '<MoveCreepersSrv-request>"
  (cl:format cl:nil "std_msgs/Bool create_new~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCreepersSrv-request)))
  "Returns full string definition for message of type 'MoveCreepersSrv-request"
  (cl:format cl:nil "std_msgs/Bool create_new~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCreepersSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'create_new))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCreepersSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCreepersSrv-request
    (cl:cons ':create_new (create_new msg))
))
;//! \htmlinclude MoveCreepersSrv-response.msg.html

(cl:defclass <MoveCreepersSrv-response> (roslisp-msg-protocol:ros-message)
  ((creeper_locations
    :reader creeper_locations
    :initarg :creeper_locations
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (reached_end
    :reader reached_end
    :initarg :reached_end
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass MoveCreepersSrv-response (<MoveCreepersSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCreepersSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCreepersSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<MoveCreepersSrv-response> is deprecated: use tower_defense-srv:MoveCreepersSrv-response instead.")))

(cl:ensure-generic-function 'creeper_locations-val :lambda-list '(m))
(cl:defmethod creeper_locations-val ((m <MoveCreepersSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:creeper_locations-val is deprecated.  Use tower_defense-srv:creeper_locations instead.")
  (creeper_locations m))

(cl:ensure-generic-function 'reached_end-val :lambda-list '(m))
(cl:defmethod reached_end-val ((m <MoveCreepersSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:reached_end-val is deprecated.  Use tower_defense-srv:reached_end instead.")
  (reached_end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCreepersSrv-response>) ostream)
  "Serializes a message object of type '<MoveCreepersSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'creeper_locations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'creeper_locations))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'reached_end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCreepersSrv-response>) istream)
  "Deserializes a message object of type '<MoveCreepersSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'creeper_locations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'creeper_locations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'reached_end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveCreepersSrv-response>)))
  "Returns string type for a service object of type '<MoveCreepersSrv-response>"
  "tower_defense/MoveCreepersSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCreepersSrv-response)))
  "Returns string type for a service object of type 'MoveCreepersSrv-response"
  "tower_defense/MoveCreepersSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveCreepersSrv-response>)))
  "Returns md5sum for a message object of type '<MoveCreepersSrv-response>"
  "f428434760fd966922080bb0858f16a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCreepersSrv-response)))
  "Returns md5sum for a message object of type 'MoveCreepersSrv-response"
  "f428434760fd966922080bb0858f16a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCreepersSrv-response>)))
  "Returns full string definition for message of type '<MoveCreepersSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point[] creeper_locations~%std_msgs/Bool reached_end~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCreepersSrv-response)))
  "Returns full string definition for message of type 'MoveCreepersSrv-response"
  (cl:format cl:nil "geometry_msgs/Point[] creeper_locations~%std_msgs/Bool reached_end~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCreepersSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'creeper_locations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'reached_end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCreepersSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCreepersSrv-response
    (cl:cons ':creeper_locations (creeper_locations msg))
    (cl:cons ':reached_end (reached_end msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveCreepersSrv)))
  'MoveCreepersSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveCreepersSrv)))
  'MoveCreepersSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCreepersSrv)))
  "Returns string type for a service object of type '<MoveCreepersSrv>"
  "tower_defense/MoveCreepersSrv")