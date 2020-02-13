; Auto-generated. Do not edit!


(cl:in-package aer1217_ardrone_vicon-msg)


;//! \htmlinclude MotorCommands.msg.html

(cl:defclass <MotorCommands> (roslisp-msg-protocol:ros-message)
  ((motor_cmd
    :reader motor_cmd
    :initarg :motor_cmd
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MotorCommands (<MotorCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aer1217_ardrone_vicon-msg:<MotorCommands> is deprecated: use aer1217_ardrone_vicon-msg:MotorCommands instead.")))

(cl:ensure-generic-function 'motor_cmd-val :lambda-list '(m))
(cl:defmethod motor_cmd-val ((m <MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:motor_cmd-val is deprecated.  Use aer1217_ardrone_vicon-msg:motor_cmd instead.")
  (motor_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommands>) ostream)
  "Serializes a message object of type '<MotorCommands>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'motor_cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommands>) istream)
  "Deserializes a message object of type '<MotorCommands>"
  (cl:setf (cl:slot-value msg 'motor_cmd) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'motor_cmd)))
    (cl:dotimes (i 4)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommands>)))
  "Returns string type for a message object of type '<MotorCommands>"
  "aer1217_ardrone_vicon/MotorCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommands)))
  "Returns string type for a message object of type 'MotorCommands"
  "aer1217_ardrone_vicon/MotorCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommands>)))
  "Returns md5sum for a message object of type '<MotorCommands>"
  "424a0da2970783751aa94563cd70d0bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommands)))
  "Returns md5sum for a message object of type 'MotorCommands"
  "424a0da2970783751aa94563cd70d0bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommands>)))
  "Returns full string definition for message of type '<MotorCommands>"
  (cl:format cl:nil "# MotorCommands~%#~%# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017~%#~%# Data communicates the motor commands in RPM as a four element array in the~%# following order [front_left, front_right, rear_left, rear_right]~%~%float64[4] motor_cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommands)))
  "Returns full string definition for message of type 'MotorCommands"
  (cl:format cl:nil "# MotorCommands~%#~%# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017~%#~%# Data communicates the motor commands in RPM as a four element array in the~%# following order [front_left, front_right, rear_left, rear_right]~%~%float64[4] motor_cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommands>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommands
    (cl:cons ':motor_cmd (motor_cmd msg))
))
