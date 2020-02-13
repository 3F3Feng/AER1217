;; Auto-generated. Do not edit!


(when (boundp 'aer1217_ardrone_simulator::ToggleCam)
  (if (not (find-package "AER1217_ARDRONE_SIMULATOR"))
    (make-package "AER1217_ARDRONE_SIMULATOR"))
  (shadow 'ToggleCam (find-package "AER1217_ARDRONE_SIMULATOR")))
(unless (find-package "AER1217_ARDRONE_SIMULATOR::TOGGLECAM")
  (make-package "AER1217_ARDRONE_SIMULATOR::TOGGLECAM"))
(unless (find-package "AER1217_ARDRONE_SIMULATOR::TOGGLECAMREQUEST")
  (make-package "AER1217_ARDRONE_SIMULATOR::TOGGLECAMREQUEST"))
(unless (find-package "AER1217_ARDRONE_SIMULATOR::TOGGLECAMRESPONSE")
  (make-package "AER1217_ARDRONE_SIMULATOR::TOGGLECAMRESPONSE"))

(in-package "ROS")





(defclass aer1217_ardrone_simulator::ToggleCamRequest
  :super ros::object
  :slots ())

(defmethod aer1217_ardrone_simulator::ToggleCamRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass aer1217_ardrone_simulator::ToggleCamResponse
  :super ros::object
  :slots (_Status ))

(defmethod aer1217_ardrone_simulator::ToggleCamResponse
  (:init
   (&key
    ((:Status __Status) "")
    )
   (send-super :init)
   (setq _Status (string __Status))
   self)
  (:Status
   (&optional __Status)
   (if __Status (setq _Status __Status)) _Status)
  (:serialization-length
   ()
   (+
    ;; string _Status
    4 (length _Status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Status
       (write-long (length _Status) s) (princ _Status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass aer1217_ardrone_simulator::ToggleCam
  :super ros::object
  :slots ())

(setf (get aer1217_ardrone_simulator::ToggleCam :md5sum-) "8fc5f5608d6072d9e82dd30f42a4228c")
(setf (get aer1217_ardrone_simulator::ToggleCam :datatype-) "aer1217_ardrone_simulator/ToggleCam")
(setf (get aer1217_ardrone_simulator::ToggleCam :request) aer1217_ardrone_simulator::ToggleCamRequest)
(setf (get aer1217_ardrone_simulator::ToggleCam :response) aer1217_ardrone_simulator::ToggleCamResponse)

(defmethod aer1217_ardrone_simulator::ToggleCamRequest
  (:response () (instance aer1217_ardrone_simulator::ToggleCamResponse :init)))

(setf (get aer1217_ardrone_simulator::ToggleCamRequest :md5sum-) "8fc5f5608d6072d9e82dd30f42a4228c")
(setf (get aer1217_ardrone_simulator::ToggleCamRequest :datatype-) "aer1217_ardrone_simulator/ToggleCamRequest")
(setf (get aer1217_ardrone_simulator::ToggleCamRequest :definition-)
      "---
string Status

")

(setf (get aer1217_ardrone_simulator::ToggleCamResponse :md5sum-) "8fc5f5608d6072d9e82dd30f42a4228c")
(setf (get aer1217_ardrone_simulator::ToggleCamResponse :datatype-) "aer1217_ardrone_simulator/ToggleCamResponse")
(setf (get aer1217_ardrone_simulator::ToggleCamResponse :definition-)
      "---
string Status

")



(provide :aer1217_ardrone_simulator/ToggleCam "8fc5f5608d6072d9e82dd30f42a4228c")


