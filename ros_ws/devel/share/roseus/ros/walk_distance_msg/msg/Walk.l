;; Auto-generated. Do not edit!


(when (boundp 'walk_distance_msg::Walk)
  (if (not (find-package "WALK_DISTANCE_MSG"))
    (make-package "WALK_DISTANCE_MSG"))
  (shadow 'Walk (find-package "WALK_DISTANCE_MSG")))
(unless (find-package "WALK_DISTANCE_MSG::WALK")
  (make-package "WALK_DISTANCE_MSG::WALK"))

(in-package "ROS")
;;//! \htmlinclude Walk.msg.html


(defclass walk_distance_msg::Walk
  :super ros::object
  :slots (_direction _distance ))

(defmethod walk_distance_msg::Walk
  (:init
   (&key
    ((:direction __direction) 0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _direction (round __direction))
   (setq _distance (float __distance))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; int32 _direction
    4
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _direction
       (write-long _direction s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _direction
     (setq _direction (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get walk_distance_msg::Walk :md5sum-) "fa939a7f5aefe801fd8d555219b82603")
(setf (get walk_distance_msg::Walk :datatype-) "walk_distance_msg/Walk")
(setf (get walk_distance_msg::Walk :definition-)
      "int32 direction
float32 distance

")



(provide :walk_distance_msg/Walk "fa939a7f5aefe801fd8d555219b82603")


