;; Auto-generated. Do not edit!


(when (boundp 'aer1217_ardrone_vicon::MotorCommands)
  (if (not (find-package "AER1217_ARDRONE_VICON"))
    (make-package "AER1217_ARDRONE_VICON"))
  (shadow 'MotorCommands (find-package "AER1217_ARDRONE_VICON")))
(unless (find-package "AER1217_ARDRONE_VICON::MOTORCOMMANDS")
  (make-package "AER1217_ARDRONE_VICON::MOTORCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude MotorCommands.msg.html


(defclass aer1217_ardrone_vicon::MotorCommands
  :super ros::object
  :slots (_motor_cmd ))

(defmethod aer1217_ardrone_vicon::MotorCommands
  (:init
   (&key
    ((:motor_cmd __motor_cmd) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _motor_cmd __motor_cmd)
   self)
  (:motor_cmd
   (&optional __motor_cmd)
   (if __motor_cmd (setq _motor_cmd __motor_cmd)) _motor_cmd)
  (:serialization-length
   ()
   (+
    ;; float64[4] _motor_cmd
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[4] _motor_cmd
     (dotimes (i 4)
       (sys::poke (elt _motor_cmd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[4] _motor_cmd
   (dotimes (i (length _motor_cmd))
     (setf (elt _motor_cmd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get aer1217_ardrone_vicon::MotorCommands :md5sum-) "424a0da2970783751aa94563cd70d0bb")
(setf (get aer1217_ardrone_vicon::MotorCommands :datatype-) "aer1217_ardrone_vicon/MotorCommands")
(setf (get aer1217_ardrone_vicon::MotorCommands :definition-)
      "# MotorCommands
#
# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017
#
# Data communicates the motor commands in RPM as a four element array in the
# following order [front_left, front_right, rear_left, rear_right]

float64[4] motor_cmd

")



(provide :aer1217_ardrone_vicon/MotorCommands "424a0da2970783751aa94563cd70d0bb")


