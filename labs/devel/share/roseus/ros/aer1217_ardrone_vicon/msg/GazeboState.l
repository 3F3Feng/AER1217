;; Auto-generated. Do not edit!


(when (boundp 'aer1217_ardrone_vicon::GazeboState)
  (if (not (find-package "AER1217_ARDRONE_VICON"))
    (make-package "AER1217_ARDRONE_VICON"))
  (shadow 'GazeboState (find-package "AER1217_ARDRONE_VICON")))
(unless (find-package "AER1217_ARDRONE_VICON::GAZEBOSTATE")
  (make-package "AER1217_ARDRONE_VICON::GAZEBOSTATE"))

(in-package "ROS")
;;//! \htmlinclude GazeboState.msg.html


(defclass aer1217_ardrone_vicon::GazeboState
  :super ros::object
  :slots (_position _quaternion ))

(defmethod aer1217_ardrone_vicon::GazeboState
  (:init
   (&key
    ((:position __position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:quaternion __quaternion) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _position __position)
   (setq _quaternion __quaternion)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:quaternion
   (&optional __quaternion)
   (if __quaternion (setq _quaternion __quaternion)) _quaternion)
  (:serialization-length
   ()
   (+
    ;; float64[3] _position
    (* 8    3)
    ;; float64[4] _quaternion
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _position
     (dotimes (i 3)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _quaternion
     (dotimes (i 4)
       (sys::poke (elt _quaternion i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _quaternion
   (dotimes (i (length _quaternion))
     (setf (elt _quaternion i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get aer1217_ardrone_vicon::GazeboState :md5sum-) "79dd5bb01ac0c08d4da2875a050d64ea")
(setf (get aer1217_ardrone_vicon::GazeboState :datatype-) "aer1217_ardrone_vicon/GazeboState")
(setf (get aer1217_ardrone_vicon::GazeboState :definition-)
      "# GazeboState
#
# Rikky Duivenvoorden 2017-01-31 -- For use in AER1217 Winter 2017
#
# Data communicates the position in a three element array [x, y, z] and the 
# rotation in the form of a quaternion in a four element array [x, y, z, w].

float64[3] position

float64[4] quaternion

")



(provide :aer1217_ardrone_vicon/GazeboState "79dd5bb01ac0c08d4da2875a050d64ea")


