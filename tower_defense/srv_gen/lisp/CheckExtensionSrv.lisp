; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude CheckExtensionSrv-request.msg.html

(cl:defclass <CheckExtensionSrv-request> (roslisp-msg-protocol:ros-message)
  ((q_near
    :reader q_near
    :initarg :q_near
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (q_new
    :reader q_new
    :initarg :q_new
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass CheckExtensionSrv-request (<CheckExtensionSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckExtensionSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckExtensionSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<CheckExtensionSrv-request> is deprecated: use tower_defense-srv:CheckExtensionSrv-request instead.")))

(cl:ensure-generic-function 'q_near-val :lambda-list '(m))
(cl:defmethod q_near-val ((m <CheckExtensionSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:q_near-val is deprecated.  Use tower_defense-srv:q_near instead.")
  (q_near m))

(cl:ensure-generic-function 'q_new-val :lambda-list '(m))
(cl:defmethod q_new-val ((m <CheckExtensionSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:q_new-val is deprecated.  Use tower_defense-srv:q_new instead.")
  (q_new m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckExtensionSrv-request>) ostream)
  "Serializes a message object of type '<CheckExtensionSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q_near) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q_new) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckExtensionSrv-request>) istream)
  "Deserializes a message object of type '<CheckExtensionSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q_near) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q_new) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckExtensionSrv-request>)))
  "Returns string type for a service object of type '<CheckExtensionSrv-request>"
  "tower_defense/CheckExtensionSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckExtensionSrv-request)))
  "Returns string type for a service object of type 'CheckExtensionSrv-request"
  "tower_defense/CheckExtensionSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckExtensionSrv-request>)))
  "Returns md5sum for a message object of type '<CheckExtensionSrv-request>"
  "103e7eef778bb10302817d90a24b89ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckExtensionSrv-request)))
  "Returns md5sum for a message object of type 'CheckExtensionSrv-request"
  "103e7eef778bb10302817d90a24b89ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckExtensionSrv-request>)))
  "Returns full string definition for message of type '<CheckExtensionSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point q_near~%geometry_msgs/Point q_new~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckExtensionSrv-request)))
  "Returns full string definition for message of type 'CheckExtensionSrv-request"
  (cl:format cl:nil "geometry_msgs/Point q_near~%geometry_msgs/Point q_new~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckExtensionSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q_near))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q_new))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckExtensionSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckExtensionSrv-request
    (cl:cons ':q_near (q_near msg))
    (cl:cons ':q_new (q_new msg))
))
;//! \htmlinclude CheckExtensionSrv-response.msg.html

(cl:defclass <CheckExtensionSrv-response> (roslisp-msg-protocol:ros-message)
  ((valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckExtensionSrv-response (<CheckExtensionSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckExtensionSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckExtensionSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<CheckExtensionSrv-response> is deprecated: use tower_defense-srv:CheckExtensionSrv-response instead.")))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <CheckExtensionSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:valid-val is deprecated.  Use tower_defense-srv:valid instead.")
  (valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckExtensionSrv-response>) ostream)
  "Serializes a message object of type '<CheckExtensionSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckExtensionSrv-response>) istream)
  "Deserializes a message object of type '<CheckExtensionSrv-response>"
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckExtensionSrv-response>)))
  "Returns string type for a service object of type '<CheckExtensionSrv-response>"
  "tower_defense/CheckExtensionSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckExtensionSrv-response)))
  "Returns string type for a service object of type 'CheckExtensionSrv-response"
  "tower_defense/CheckExtensionSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckExtensionSrv-response>)))
  "Returns md5sum for a message object of type '<CheckExtensionSrv-response>"
  "103e7eef778bb10302817d90a24b89ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckExtensionSrv-response)))
  "Returns md5sum for a message object of type 'CheckExtensionSrv-response"
  "103e7eef778bb10302817d90a24b89ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckExtensionSrv-response>)))
  "Returns full string definition for message of type '<CheckExtensionSrv-response>"
  (cl:format cl:nil "bool valid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckExtensionSrv-response)))
  "Returns full string definition for message of type 'CheckExtensionSrv-response"
  (cl:format cl:nil "bool valid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckExtensionSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckExtensionSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckExtensionSrv-response
    (cl:cons ':valid (valid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckExtensionSrv)))
  'CheckExtensionSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckExtensionSrv)))
  'CheckExtensionSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckExtensionSrv)))
  "Returns string type for a service object of type '<CheckExtensionSrv>"
  "tower_defense/CheckExtensionSrv")