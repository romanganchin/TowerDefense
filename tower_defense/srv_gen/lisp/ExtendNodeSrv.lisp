; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude ExtendNodeSrv-request.msg.html

(cl:defclass <ExtendNodeSrv-request> (roslisp-msg-protocol:ros-message)
  ((P
    :reader P
    :initarg :P
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (q
    :reader q
    :initarg :q
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass ExtendNodeSrv-request (<ExtendNodeSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExtendNodeSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExtendNodeSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<ExtendNodeSrv-request> is deprecated: use tower_defense-srv:ExtendNodeSrv-request instead.")))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <ExtendNodeSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:P-val is deprecated.  Use tower_defense-srv:P instead.")
  (P m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <ExtendNodeSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:q-val is deprecated.  Use tower_defense-srv:q instead.")
  (q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExtendNodeSrv-request>) ostream)
  "Serializes a message object of type '<ExtendNodeSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'P))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExtendNodeSrv-request>) istream)
  "Deserializes a message object of type '<ExtendNodeSrv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'P) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'P)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExtendNodeSrv-request>)))
  "Returns string type for a service object of type '<ExtendNodeSrv-request>"
  "tower_defense/ExtendNodeSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtendNodeSrv-request)))
  "Returns string type for a service object of type 'ExtendNodeSrv-request"
  "tower_defense/ExtendNodeSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExtendNodeSrv-request>)))
  "Returns md5sum for a message object of type '<ExtendNodeSrv-request>"
  "39895a811c277641b38420f358a62ecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExtendNodeSrv-request)))
  "Returns md5sum for a message object of type 'ExtendNodeSrv-request"
  "39895a811c277641b38420f358a62ecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExtendNodeSrv-request>)))
  "Returns full string definition for message of type '<ExtendNodeSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point[] P~%geometry_msgs/Point q~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExtendNodeSrv-request)))
  "Returns full string definition for message of type 'ExtendNodeSrv-request"
  (cl:format cl:nil "geometry_msgs/Point[] P~%geometry_msgs/Point q~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExtendNodeSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExtendNodeSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExtendNodeSrv-request
    (cl:cons ':P (P msg))
    (cl:cons ':q (q msg))
))
;//! \htmlinclude ExtendNodeSrv-response.msg.html

(cl:defclass <ExtendNodeSrv-response> (roslisp-msg-protocol:ros-message)
  ((q_near_index
    :reader q_near_index
    :initarg :q_near_index
    :type cl:integer
    :initform 0)
   (q_new
    :reader q_new
    :initarg :q_new
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass ExtendNodeSrv-response (<ExtendNodeSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExtendNodeSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExtendNodeSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<ExtendNodeSrv-response> is deprecated: use tower_defense-srv:ExtendNodeSrv-response instead.")))

(cl:ensure-generic-function 'q_near_index-val :lambda-list '(m))
(cl:defmethod q_near_index-val ((m <ExtendNodeSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:q_near_index-val is deprecated.  Use tower_defense-srv:q_near_index instead.")
  (q_near_index m))

(cl:ensure-generic-function 'q_new-val :lambda-list '(m))
(cl:defmethod q_new-val ((m <ExtendNodeSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:q_new-val is deprecated.  Use tower_defense-srv:q_new instead.")
  (q_new m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExtendNodeSrv-response>) ostream)
  "Serializes a message object of type '<ExtendNodeSrv-response>"
  (cl:let* ((signed (cl:slot-value msg 'q_near_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q_new) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExtendNodeSrv-response>) istream)
  "Deserializes a message object of type '<ExtendNodeSrv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'q_near_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q_new) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExtendNodeSrv-response>)))
  "Returns string type for a service object of type '<ExtendNodeSrv-response>"
  "tower_defense/ExtendNodeSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtendNodeSrv-response)))
  "Returns string type for a service object of type 'ExtendNodeSrv-response"
  "tower_defense/ExtendNodeSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExtendNodeSrv-response>)))
  "Returns md5sum for a message object of type '<ExtendNodeSrv-response>"
  "39895a811c277641b38420f358a62ecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExtendNodeSrv-response)))
  "Returns md5sum for a message object of type 'ExtendNodeSrv-response"
  "39895a811c277641b38420f358a62ecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExtendNodeSrv-response>)))
  "Returns full string definition for message of type '<ExtendNodeSrv-response>"
  (cl:format cl:nil "int32 q_near_index~%geometry_msgs/Point q_new~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExtendNodeSrv-response)))
  "Returns full string definition for message of type 'ExtendNodeSrv-response"
  (cl:format cl:nil "int32 q_near_index~%geometry_msgs/Point q_new~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExtendNodeSrv-response>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q_new))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExtendNodeSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExtendNodeSrv-response
    (cl:cons ':q_near_index (q_near_index msg))
    (cl:cons ':q_new (q_new msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExtendNodeSrv)))
  'ExtendNodeSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExtendNodeSrv)))
  'ExtendNodeSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtendNodeSrv)))
  "Returns string type for a service object of type '<ExtendNodeSrv>"
  "tower_defense/ExtendNodeSrv")