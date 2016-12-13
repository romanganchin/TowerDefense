; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude HurtCreeperSrv-request.msg.html

(cl:defclass <HurtCreeperSrv-request> (roslisp-msg-protocol:ros-message)
  ((damage
    :reader damage
    :initarg :damage
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (location
    :reader location
    :initarg :location
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass HurtCreeperSrv-request (<HurtCreeperSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HurtCreeperSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HurtCreeperSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<HurtCreeperSrv-request> is deprecated: use tower_defense-srv:HurtCreeperSrv-request instead.")))

(cl:ensure-generic-function 'damage-val :lambda-list '(m))
(cl:defmethod damage-val ((m <HurtCreeperSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:damage-val is deprecated.  Use tower_defense-srv:damage instead.")
  (damage m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <HurtCreeperSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:location-val is deprecated.  Use tower_defense-srv:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HurtCreeperSrv-request>) ostream)
  "Serializes a message object of type '<HurtCreeperSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'damage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'damage))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HurtCreeperSrv-request>) istream)
  "Deserializes a message object of type '<HurtCreeperSrv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'damage) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'damage)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'location) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'location)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HurtCreeperSrv-request>)))
  "Returns string type for a service object of type '<HurtCreeperSrv-request>"
  "tower_defense/HurtCreeperSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HurtCreeperSrv-request)))
  "Returns string type for a service object of type 'HurtCreeperSrv-request"
  "tower_defense/HurtCreeperSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HurtCreeperSrv-request>)))
  "Returns md5sum for a message object of type '<HurtCreeperSrv-request>"
  "ab1e0c29d62ef3646b99606e9c08aed0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HurtCreeperSrv-request)))
  "Returns md5sum for a message object of type 'HurtCreeperSrv-request"
  "ab1e0c29d62ef3646b99606e9c08aed0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HurtCreeperSrv-request>)))
  "Returns full string definition for message of type '<HurtCreeperSrv-request>"
  (cl:format cl:nil "int32[] damage~%geometry_msgs/Point[] location~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HurtCreeperSrv-request)))
  "Returns full string definition for message of type 'HurtCreeperSrv-request"
  (cl:format cl:nil "int32[] damage~%geometry_msgs/Point[] location~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HurtCreeperSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'damage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'location) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HurtCreeperSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HurtCreeperSrv-request
    (cl:cons ':damage (damage msg))
    (cl:cons ':location (location msg))
))
;//! \htmlinclude HurtCreeperSrv-response.msg.html

(cl:defclass <HurtCreeperSrv-response> (roslisp-msg-protocol:ros-message)
  ((creeper_locations
    :reader creeper_locations
    :initarg :creeper_locations
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass HurtCreeperSrv-response (<HurtCreeperSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HurtCreeperSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HurtCreeperSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<HurtCreeperSrv-response> is deprecated: use tower_defense-srv:HurtCreeperSrv-response instead.")))

(cl:ensure-generic-function 'creeper_locations-val :lambda-list '(m))
(cl:defmethod creeper_locations-val ((m <HurtCreeperSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:creeper_locations-val is deprecated.  Use tower_defense-srv:creeper_locations instead.")
  (creeper_locations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HurtCreeperSrv-response>) ostream)
  "Serializes a message object of type '<HurtCreeperSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'creeper_locations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'creeper_locations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HurtCreeperSrv-response>) istream)
  "Deserializes a message object of type '<HurtCreeperSrv-response>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HurtCreeperSrv-response>)))
  "Returns string type for a service object of type '<HurtCreeperSrv-response>"
  "tower_defense/HurtCreeperSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HurtCreeperSrv-response)))
  "Returns string type for a service object of type 'HurtCreeperSrv-response"
  "tower_defense/HurtCreeperSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HurtCreeperSrv-response>)))
  "Returns md5sum for a message object of type '<HurtCreeperSrv-response>"
  "ab1e0c29d62ef3646b99606e9c08aed0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HurtCreeperSrv-response)))
  "Returns md5sum for a message object of type 'HurtCreeperSrv-response"
  "ab1e0c29d62ef3646b99606e9c08aed0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HurtCreeperSrv-response>)))
  "Returns full string definition for message of type '<HurtCreeperSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point[] creeper_locations~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HurtCreeperSrv-response)))
  "Returns full string definition for message of type 'HurtCreeperSrv-response"
  (cl:format cl:nil "geometry_msgs/Point[] creeper_locations~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HurtCreeperSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'creeper_locations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HurtCreeperSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HurtCreeperSrv-response
    (cl:cons ':creeper_locations (creeper_locations msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HurtCreeperSrv)))
  'HurtCreeperSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HurtCreeperSrv)))
  'HurtCreeperSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HurtCreeperSrv)))
  "Returns string type for a service object of type '<HurtCreeperSrv>"
  "tower_defense/HurtCreeperSrv")