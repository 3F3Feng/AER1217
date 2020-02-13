;; Auto-generated. Do not edit!


(when (boundp 'aer1217_ardrone_vicon::DesiredStateMsg)
  (if (not (find-package "AER1217_ARDRONE_VICON"))
    (make-package "AER1217_ARDRONE_VICON"))
  (shadow 'DesiredStateMsg (find-package "AER1217_ARDRONE_VICON")))
(unless (find-package "AER1217_ARDRONE_VICON::DESIREDSTATEMSG")
  (make-package "AER1217_ARDRONE_VICON::DESIREDSTATEMSG"))

(in-package "ROS")
;;//! \htmlinclude DesiredStateMsg.msg.html


(defclass aer1217_ardrone_vicon::DesiredStateMsg
  :super ros::object
  :slots (_x _x_dot _yaw _yaw_rate ))

(defmethod aer1217_ardrone_vicon::DesiredStateMsg
  (:init
   (&key
    ((:x __x) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:x_dot __x_dot) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    )
   (send-super :init)
   (setq _x __x)
   (setq _x_dot __x_dot)
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:x_dot
   (&optional __x_dot)
   (if __x_dot (setq _x_dot __x_dot)) _x_dot)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:serialization-length
   ()
   (+
    ;; float64[3] _x
    (* 8    3)
    ;; float64[3] _x_dot
    (* 8    3)
    ;; float64 _yaw
    8
    ;; float64 _yaw_rate
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _x
     (dotimes (i 3)
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _x_dot
     (dotimes (i 3)
       (sys::poke (elt _x_dot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _x
   (dotimes (i (length _x))
     (setf (elt _x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _x_dot
   (dotimes (i (length _x_dot))
     (setf (elt _x_dot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get aer1217_ardrone_vicon::DesiredStateMsg :md5sum-) "cc61550db4d69f891417c4905802aea9")
(setf (get aer1217_ardrone_vicon::DesiredStateMsg :datatype-) "aer1217_ardrone_vicon/DesiredStateMsg")
(setf (get aer1217_ardrone_vicon::DesiredStateMsg :definition-)
      "# MotorCommands
#
# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017
#
# Data communicates the motor commands in RPM as a four element array in the
# following order [front_left, front_right, rear_left, rear_right]

float64[3] x
float64[3] x_dot
float64 yaw
float64 yaw_rate

")



(provide :aer1217_ardrone_vicon/DesiredStateMsg "cc61550db4d69f891417c4905802aea9")


