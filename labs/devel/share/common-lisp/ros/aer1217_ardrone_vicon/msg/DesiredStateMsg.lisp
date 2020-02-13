; Auto-generated. Do not edit!


(cl:in-package aer1217_ardrone_vicon-msg)


;//! \htmlinclude DesiredStateMsg.msg.html

(cl:defclass <DesiredStateMsg> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (x_dot
    :reader x_dot
    :initarg :x_dot
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass DesiredStateMsg (<DesiredStateMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DesiredStateMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DesiredStateMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aer1217_ardrone_vicon-msg:<DesiredStateMsg> is deprecated: use aer1217_ardrone_vicon-msg:DesiredStateMsg instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <DesiredStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:x-val is deprecated.  Use aer1217_ardrone_vicon-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'x_dot-val :lambda-list '(m))
(cl:defmethod x_dot-val ((m <DesiredStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:x_dot-val is deprecated.  Use aer1217_ardrone_vicon-msg:x_dot instead.")
  (x_dot m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <DesiredStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:yaw-val is deprecated.  Use aer1217_ardrone_vicon-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <DesiredStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:yaw_rate-val is deprecated.  Use aer1217_ardrone_vicon-msg:yaw_rate instead.")
  (yaw_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DesiredStateMsg>) ostream)
  "Serializes a message object of type '<DesiredStateMsg>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x_dot))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DesiredStateMsg>) istream)
  "Deserializes a message object of type '<DesiredStateMsg>"
  (cl:setf (cl:slot-value msg 'x) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'x)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'x_dot) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'x_dot)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DesiredStateMsg>)))
  "Returns string type for a message object of type '<DesiredStateMsg>"
  "aer1217_ardrone_vicon/DesiredStateMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DesiredStateMsg)))
  "Returns string type for a message object of type 'DesiredStateMsg"
  "aer1217_ardrone_vicon/DesiredStateMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DesiredStateMsg>)))
  "Returns md5sum for a message object of type '<DesiredStateMsg>"
  "cc61550db4d69f891417c4905802aea9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DesiredStateMsg)))
  "Returns md5sum for a message object of type 'DesiredStateMsg"
  "cc61550db4d69f891417c4905802aea9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DesiredStateMsg>)))
  "Returns full string definition for message of type '<DesiredStateMsg>"
  (cl:format cl:nil "# MotorCommands~%#~%# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017~%#~%# Data communicates the motor commands in RPM as a four element array in the~%# following order [front_left, front_right, rear_left, rear_right]~%~%float64[3] x~%float64[3] x_dot~%float64 yaw~%float64 yaw_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DesiredStateMsg)))
  "Returns full string definition for message of type 'DesiredStateMsg"
  (cl:format cl:nil "# MotorCommands~%#~%# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017~%#~%# Data communicates the motor commands in RPM as a four element array in the~%# following order [front_left, front_right, rear_left, rear_right]~%~%float64[3] x~%float64[3] x_dot~%float64 yaw~%float64 yaw_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DesiredStateMsg>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x_dot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DesiredStateMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'DesiredStateMsg
    (cl:cons ':x (x msg))
    (cl:cons ':x_dot (x_dot msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
))
