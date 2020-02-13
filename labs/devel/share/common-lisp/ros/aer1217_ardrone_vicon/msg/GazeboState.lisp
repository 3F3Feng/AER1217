; Auto-generated. Do not edit!


(cl:in-package aer1217_ardrone_vicon-msg)


;//! \htmlinclude GazeboState.msg.html

(cl:defclass <GazeboState> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (quaternion
    :reader quaternion
    :initarg :quaternion
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GazeboState (<GazeboState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GazeboState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GazeboState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aer1217_ardrone_vicon-msg:<GazeboState> is deprecated: use aer1217_ardrone_vicon-msg:GazeboState instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <GazeboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:position-val is deprecated.  Use aer1217_ardrone_vicon-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'quaternion-val :lambda-list '(m))
(cl:defmethod quaternion-val ((m <GazeboState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aer1217_ardrone_vicon-msg:quaternion-val is deprecated.  Use aer1217_ardrone_vicon-msg:quaternion instead.")
  (quaternion m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GazeboState>) ostream)
  "Serializes a message object of type '<GazeboState>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'position))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'quaternion))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GazeboState>) istream)
  "Deserializes a message object of type '<GazeboState>"
  (cl:setf (cl:slot-value msg 'position) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'position)))
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
  (cl:setf (cl:slot-value msg 'quaternion) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'quaternion)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GazeboState>)))
  "Returns string type for a message object of type '<GazeboState>"
  "aer1217_ardrone_vicon/GazeboState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GazeboState)))
  "Returns string type for a message object of type 'GazeboState"
  "aer1217_ardrone_vicon/GazeboState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GazeboState>)))
  "Returns md5sum for a message object of type '<GazeboState>"
  "79dd5bb01ac0c08d4da2875a050d64ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GazeboState)))
  "Returns md5sum for a message object of type 'GazeboState"
  "79dd5bb01ac0c08d4da2875a050d64ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GazeboState>)))
  "Returns full string definition for message of type '<GazeboState>"
  (cl:format cl:nil "# GazeboState~%#~%# Rikky Duivenvoorden 2017-01-31 -- For use in AER1217 Winter 2017~%#~%# Data communicates the position in a three element array [x, y, z] and the ~%# rotation in the form of a quaternion in a four element array [x, y, z, w].~%~%float64[3] position~%~%float64[4] quaternion~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GazeboState)))
  "Returns full string definition for message of type 'GazeboState"
  (cl:format cl:nil "# GazeboState~%#~%# Rikky Duivenvoorden 2017-01-31 -- For use in AER1217 Winter 2017~%#~%# Data communicates the position in a three element array [x, y, z] and the ~%# rotation in the form of a quaternion in a four element array [x, y, z, w].~%~%float64[3] position~%~%float64[4] quaternion~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GazeboState>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quaternion) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GazeboState>))
  "Converts a ROS message object to a list"
  (cl:list 'GazeboState
    (cl:cons ':position (position msg))
    (cl:cons ':quaternion (quaternion msg))
))
