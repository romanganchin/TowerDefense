; Auto-generated. Do not edit!


(cl:in-package tower_defense-msg)


;//! \htmlinclude RRTNode.msg.html

(cl:defclass <RRTNode> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (parent
    :reader parent
    :initarg :parent
    :type cl:integer
    :initform 0))
)

(cl:defclass RRTNode (<RRTNode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RRTNode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RRTNode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tower_defense-msg:<RRTNode> is deprecated: use tower_defense-msg:RRTNode instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <RRTNode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-msg:location-val is deprecated.  Use tower_defense-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'parent-val :lambda-list '(m))
(cl:defmethod parent-val ((m <RRTNode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tower_defense-msg:parent-val is deprecated.  Use tower_defense-msg:parent instead.")
  (parent m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RRTNode>) ostream)
  "Serializes a message object of type '<RRTNode>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
  (cl:let* ((signed (cl:slot-value msg 'parent)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RRTNode>) istream)
  "Deserializes a message object of type '<RRTNode>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'parent) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RRTNode>)))
  "Returns string type for a message object of type '<RRTNode>"
  "tower_defense/RRTNode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RRTNode)))
  "Returns string type for a message object of type 'RRTNode"
  "tower_defense/RRTNode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RRTNode>)))
  "Returns md5sum for a message object of type '<RRTNode>"
  "b537b82f1208ce6fb70806424f02548b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RRTNode)))
  "Returns md5sum for a message object of type 'RRTNode"
  "b537b82f1208ce6fb70806424f02548b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RRTNode>)))
  "Returns full string definition for message of type '<RRTNode>"
  (cl:format cl:nil "# Location of the tree node.~%geometry_msgs/Point location~%~%# Index of parent node, -1 if there is no parent (root node).~%int32 parent~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RRTNode)))
  "Returns full string definition for message of type 'RRTNode"
  (cl:format cl:nil "# Location of the tree node.~%geometry_msgs/Point location~%~%# Index of parent node, -1 if there is no parent (root node).~%int32 parent~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RRTNode>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RRTNode>))
  "Converts a ROS message object to a list"
  (cl:list 'RRTNode
    (cl:cons ':location (location msg))
    (cl:cons ':parent (parent msg))
))
