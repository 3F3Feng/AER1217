; Auto-generated. Do not edit!


(cl:in-package aer1217_ardrone_simulator-srv)


;//! \htmlinclude ToggleCam-request.msg.html

(cl:defclass <ToggleCam-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ToggleCam-request (<ToggleCam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToggleCam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToggleCam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aer1217_ardrone_simulator-srv:<ToggleCam-request> is deprecated: use aer1217_ardrone_simulator-srv:ToggleCam-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToggleCam-request>) ostream)
  "Serializes a message object of type '<ToggleCam-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToggleCam-request>) istream)
  "Deserializes a message object of type '<ToggleCam-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToggleCam-request>)))
  "Returns string type for a service object of type '<ToggleCam-request>"
  "aer1217_ardrone_simulator/ToggleCamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleCam-request)))
  "Returns string type for a service object of type 'ToggleCam-request"
  "aer1217_ardrone_simulator/ToggleCamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToggleCam-request>)))
  "Returns md5sum for a message object of type '<ToggleCam-request>"
  "8fc5f5608d6072d9e82dd30f42a4228c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToggleCam-request)))
  "Returns md5sum for a message object of type 'ToggleCam-request"
  "8fc5f5608d6072d9e82dd30f42a4228c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToggleCam-request>)))
  "Returns full string definition for message of type '<ToggleCam-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToggleCam-request)))
  "Returns full string definition for message of type 'ToggleCam-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToggleCam-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToggleCam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ToggleCam-request
))
;//! \htmlinclude ToggleCam-response.msg.html

(cl:defclass <ToggleCam-response> (roslisp-msg-protocol:ros-message)
  ((Status
    :reader Status
    :initarg :Status
    :type cl:string
    :initform ""))
)

(cl:defclass ToggleCam-response (<ToggleCam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToggleCam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToggleCam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aer1217_ardrone_simulator-srv:<ToggleCam-response> is deprecated: use aer1217_ardrone_simulator-srv:ToggleCam-response instead.")))

(cl:ensure-generic-function 'Status-val :lambda-list '(m))
(cl:defmethod Status-val ((m <ToggleCam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_simulator-srv:Status-val is deprecated.  Use aer1217_ardrone_simulator-srv:Status instead.")
  (Status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToggleCam-response>) ostream)
  "Serializes a message object of type '<ToggleCam-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToggleCam-response>) istream)
  "Deserializes a message object of type '<ToggleCam-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToggleCam-response>)))
  "Returns string type for a service object of type '<ToggleCam-response>"
  "aer1217_ardrone_simulator/ToggleCamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleCam-response)))
  "Returns string type for a service object of type 'ToggleCam-response"
  "aer1217_ardrone_simulator/ToggleCamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToggleCam-response>)))
  "Returns md5sum for a message object of type '<ToggleCam-response>"
  "8fc5f5608d6072d9e82dd30f42a4228c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToggleCam-response)))
  "Returns md5sum for a message object of type 'ToggleCam-response"
  "8fc5f5608d6072d9e82dd30f42a4228c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToggleCam-response>)))
  "Returns full string definition for message of type '<ToggleCam-response>"
  (cl:format cl:nil "string Status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToggleCam-response)))
  "Returns full string definition for message of type 'ToggleCam-response"
  (cl:format cl:nil "string Status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToggleCam-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToggleCam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ToggleCam-response
    (cl:cons ':Status (Status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ToggleCam)))
  'ToggleCam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ToggleCam)))
  'ToggleCam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleCam)))
  "Returns string type for a service object of type '<ToggleCam>"
  "aer1217_ardrone_simulator/ToggleCam")