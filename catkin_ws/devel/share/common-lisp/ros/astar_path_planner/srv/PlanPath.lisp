; Auto-generated. Do not edit!


(cl:in-package astar_path_planner-srv)


;//! \htmlinclude PlanPath-request.msg.html

(cl:defclass <PlanPath-request> (roslisp-msg-protocol:ros-message)
  ((diagonal_movement
    :reader diagonal_movement
    :initarg :diagonal_movement
    :type cl:boolean
    :initform cl:nil)
   (heuristic_cost_weight
    :reader heuristic_cost_weight
    :initarg :heuristic_cost_weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass PlanPath-request (<PlanPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name astar_path_planner-srv:<PlanPath-request> is deprecated: use astar_path_planner-srv:PlanPath-request instead.")))

(cl:ensure-generic-function 'diagonal_movement-val :lambda-list '(m))
(cl:defmethod diagonal_movement-val ((m <PlanPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader astar_path_planner-srv:diagonal_movement-val is deprecated.  Use astar_path_planner-srv:diagonal_movement instead.")
  (diagonal_movement m))

(cl:ensure-generic-function 'heuristic_cost_weight-val :lambda-list '(m))
(cl:defmethod heuristic_cost_weight-val ((m <PlanPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader astar_path_planner-srv:heuristic_cost_weight-val is deprecated.  Use astar_path_planner-srv:heuristic_cost_weight instead.")
  (heuristic_cost_weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanPath-request>) ostream)
  "Serializes a message object of type '<PlanPath-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'diagonal_movement) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heuristic_cost_weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanPath-request>) istream)
  "Deserializes a message object of type '<PlanPath-request>"
    (cl:setf (cl:slot-value msg 'diagonal_movement) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heuristic_cost_weight) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanPath-request>)))
  "Returns string type for a service object of type '<PlanPath-request>"
  "astar_path_planner/PlanPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanPath-request)))
  "Returns string type for a service object of type 'PlanPath-request"
  "astar_path_planner/PlanPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanPath-request>)))
  "Returns md5sum for a message object of type '<PlanPath-request>"
  "64724dde033fbe8f19a82fd1a78b4bdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanPath-request)))
  "Returns md5sum for a message object of type 'PlanPath-request"
  "64724dde033fbe8f19a82fd1a78b4bdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanPath-request>)))
  "Returns full string definition for message of type '<PlanPath-request>"
  (cl:format cl:nil "bool diagonal_movement~%float64 heuristic_cost_weight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanPath-request)))
  "Returns full string definition for message of type 'PlanPath-request"
  (cl:format cl:nil "bool diagonal_movement~%float64 heuristic_cost_weight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanPath-request>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanPath-request
    (cl:cons ':diagonal_movement (diagonal_movement msg))
    (cl:cons ':heuristic_cost_weight (heuristic_cost_weight msg))
))
;//! \htmlinclude PlanPath-response.msg.html

(cl:defclass <PlanPath-response> (roslisp-msg-protocol:ros-message)
  ((length_of_path
    :reader length_of_path
    :initarg :length_of_path
    :type cl:integer
    :initform 0)
   (number_of_nodes_in_closed_set
    :reader number_of_nodes_in_closed_set
    :initarg :number_of_nodes_in_closed_set
    :type cl:integer
    :initform 0))
)

(cl:defclass PlanPath-response (<PlanPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name astar_path_planner-srv:<PlanPath-response> is deprecated: use astar_path_planner-srv:PlanPath-response instead.")))

(cl:ensure-generic-function 'length_of_path-val :lambda-list '(m))
(cl:defmethod length_of_path-val ((m <PlanPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader astar_path_planner-srv:length_of_path-val is deprecated.  Use astar_path_planner-srv:length_of_path instead.")
  (length_of_path m))

(cl:ensure-generic-function 'number_of_nodes_in_closed_set-val :lambda-list '(m))
(cl:defmethod number_of_nodes_in_closed_set-val ((m <PlanPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader astar_path_planner-srv:number_of_nodes_in_closed_set-val is deprecated.  Use astar_path_planner-srv:number_of_nodes_in_closed_set instead.")
  (number_of_nodes_in_closed_set m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanPath-response>) ostream)
  "Serializes a message object of type '<PlanPath-response>"
  (cl:let* ((signed (cl:slot-value msg 'length_of_path)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'number_of_nodes_in_closed_set)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanPath-response>) istream)
  "Deserializes a message object of type '<PlanPath-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length_of_path) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number_of_nodes_in_closed_set) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanPath-response>)))
  "Returns string type for a service object of type '<PlanPath-response>"
  "astar_path_planner/PlanPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanPath-response)))
  "Returns string type for a service object of type 'PlanPath-response"
  "astar_path_planner/PlanPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanPath-response>)))
  "Returns md5sum for a message object of type '<PlanPath-response>"
  "64724dde033fbe8f19a82fd1a78b4bdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanPath-response)))
  "Returns md5sum for a message object of type 'PlanPath-response"
  "64724dde033fbe8f19a82fd1a78b4bdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanPath-response>)))
  "Returns full string definition for message of type '<PlanPath-response>"
  (cl:format cl:nil "int32 length_of_path~%int32 number_of_nodes_in_closed_set~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanPath-response)))
  "Returns full string definition for message of type 'PlanPath-response"
  (cl:format cl:nil "int32 length_of_path~%int32 number_of_nodes_in_closed_set~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanPath-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanPath-response
    (cl:cons ':length_of_path (length_of_path msg))
    (cl:cons ':number_of_nodes_in_closed_set (number_of_nodes_in_closed_set msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanPath)))
  'PlanPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanPath)))
  'PlanPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanPath)))
  "Returns string type for a service object of type '<PlanPath>"
  "astar_path_planner/PlanPath")