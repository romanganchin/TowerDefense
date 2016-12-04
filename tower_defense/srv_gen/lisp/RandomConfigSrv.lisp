; Auto-generated. Do not edit!


(cl:in-package tower_defense-srv)


;//! \htmlinclude RandomConfigSrv-request.msg.html

(cl:defclass <RandomConfigSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RandomConfigSrv-request (<RandomConfigSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RandomConfigSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RandomConfigSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<RandomConfigSrv-request> is deprecated: use tower_defense-srv:RandomConfigSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RandomConfigSrv-request>) ostream)
  "Serializes a message object of type '<RandomConfigSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RandomConfigSrv-request>) istream)
  "Deserializes a message object of type '<RandomConfigSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RandomConfigSrv-request>)))
  "Returns string type for a service object of type '<RandomConfigSrv-request>"
  "tower_defense/RandomConfigSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RandomConfigSrv-request)))
  "Returns string type for a service object of type 'RandomConfigSrv-request"
  "tower_defense/RandomConfigSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RandomConfigSrv-request>)))
  "Returns md5sum for a message object of type '<RandomConfigSrv-request>"
  "ebedd2ab5ada3637b4212236735875be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RandomConfigSrv-request)))
  "Returns md5sum for a message object of type 'RandomConfigSrv-request"
  "ebedd2ab5ada3637b4212236735875be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RandomConfigSrv-request>)))
  "Returns full string definition for message of type '<RandomConfigSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RandomConfigSrv-request)))
  "Returns full string definition for message of type 'RandomConfigSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RandomConfigSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RandomConfigSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RandomConfigSrv-request
))
;//! \htmlinclude RandomConfigSrv-response.msg.html

(cl:defclass <RandomConfigSrv-response> (roslisp-msg-protocol:ros-message)
  ((q_rand
    :reader q_rand
    :initarg :q_rand
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass RandomConfigSrv-response (<RandomConfigSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RandomConfigSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RandomConfigSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<RandomConfigSrv-response> is deprecated: use tower_defense-srv:RandomConfigSrv-response instead.")))

(cl:ensure-generic-function 'q_rand-val :lambda-list '(m))
(cl:defmethod q_rand-val ((m <RandomConfigSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-srv:q_rand-val is deprecated.  Use tower_defense-srv:q_rand instead.")
  (q_rand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RandomConfigSrv-response>) ostream)
  "Serializes a message object of type '<RandomConfigSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q_rand) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RandomConfigSrv-response>) istream)
  "Deserializes a message object of type '<RandomConfigSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q_rand) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RandomConfigSrv-response>)))
  "Returns string type for a service object of type '<RandomConfigSrv-response>"
  "tower_defense/RandomConfigSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RandomConfigSrv-response)))
  "Returns string type for a service object of type 'RandomConfigSrv-response"
  "tower_defense/RandomConfigSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RandomConfigSrv-response>)))
  "Returns md5sum for a message object of type '<RandomConfigSrv-response>"
  "ebedd2ab5ada3637b4212236735875be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RandomConfigSrv-response)))
  "Returns md5sum for a message object of type 'RandomConfigSrv-response"
  "ebedd2ab5ada3637b4212236735875be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RandomConfigSrv-response>)))
  "Returns full string definition for message of type '<RandomConfigSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point q_rand~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RandomConfigSrv-response)))
  "Returns full string definition for message of type 'RandomConfigSrv-response"
  (cl:format cl:nil "geometry_msgs/Point q_rand~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RandomConfigSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q_rand))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RandomConfigSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RandomConfigSrv-response
    (cl:cons ':q_rand (q_rand msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RandomConfigSrv)))
  'RandomConfigSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RandomConfigSrv)))
  'RandomConfigSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RandomConfigSrv)))
  "Returns string type for a service object of type '<RandomConfigSrv>"
  "tower_defense/RandomConfigSrv")