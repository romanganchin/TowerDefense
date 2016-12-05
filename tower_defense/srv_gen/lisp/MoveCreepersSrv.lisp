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
  "45228e9cc51bf84db6a86e638e9ab23d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCreepersSrv-request)))
  "Returns md5sum for a message object of type 'MoveCreepersSrv-request"
  "45228e9cc51bf84db6a86e638e9ab23d")
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
  ()
)

(cl:defclass MoveCreepersSrv-response (<MoveCreepersSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCreepersSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCreepersSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-srv:<MoveCreepersSrv-response> is deprecated: use tower_defense-srv:MoveCreepersSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCreepersSrv-response>) ostream)
  "Serializes a message object of type '<MoveCreepersSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCreepersSrv-response>) istream)
  "Deserializes a message object of type '<MoveCreepersSrv-response>"
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
  "45228e9cc51bf84db6a86e638e9ab23d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCreepersSrv-response)))
  "Returns md5sum for a message object of type 'MoveCreepersSrv-response"
  "45228e9cc51bf84db6a86e638e9ab23d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCreepersSrv-response>)))
  "Returns full string definition for message of type '<MoveCreepersSrv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCreepersSrv-response)))
  "Returns full string definition for message of type 'MoveCreepersSrv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCreepersSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCreepersSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCreepersSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveCreepersSrv)))
  'MoveCreepersSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveCreepersSrv)))
  'MoveCreepersSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCreepersSrv)))
  "Returns string type for a service object of type '<MoveCreepersSrv>"
  "tower_defense/MoveCreepersSrv")