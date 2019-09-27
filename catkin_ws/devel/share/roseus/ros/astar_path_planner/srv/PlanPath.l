;; Auto-generated. Do not edit!


(when (boundp 'astar_path_planner::PlanPath)
  (if (not (find-package "ASTAR_PATH_PLANNER"))
    (make-package "ASTAR_PATH_PLANNER"))
  (shadow 'PlanPath (find-package "ASTAR_PATH_PLANNER")))
(unless (find-package "ASTAR_PATH_PLANNER::PLANPATH")
  (make-package "ASTAR_PATH_PLANNER::PLANPATH"))
(unless (find-package "ASTAR_PATH_PLANNER::PLANPATHREQUEST")
  (make-package "ASTAR_PATH_PLANNER::PLANPATHREQUEST"))
(unless (find-package "ASTAR_PATH_PLANNER::PLANPATHRESPONSE")
  (make-package "ASTAR_PATH_PLANNER::PLANPATHRESPONSE"))

(in-package "ROS")





(defclass astar_path_planner::PlanPathRequest
  :super ros::object
  :slots (_diagonal_movement _heuristic_cost_weight ))

(defmethod astar_path_planner::PlanPathRequest
  (:init
   (&key
    ((:diagonal_movement __diagonal_movement) nil)
    ((:heuristic_cost_weight __heuristic_cost_weight) 0.0)
    )
   (send-super :init)
   (setq _diagonal_movement __diagonal_movement)
   (setq _heuristic_cost_weight (float __heuristic_cost_weight))
   self)
  (:diagonal_movement
   (&optional __diagonal_movement)
   (if __diagonal_movement (setq _diagonal_movement __diagonal_movement)) _diagonal_movement)
  (:heuristic_cost_weight
   (&optional __heuristic_cost_weight)
   (if __heuristic_cost_weight (setq _heuristic_cost_weight __heuristic_cost_weight)) _heuristic_cost_weight)
  (:serialization-length
   ()
   (+
    ;; bool _diagonal_movement
    1
    ;; float64 _heuristic_cost_weight
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _diagonal_movement
       (if _diagonal_movement (write-byte -1 s) (write-byte 0 s))
     ;; float64 _heuristic_cost_weight
       (sys::poke _heuristic_cost_weight (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _diagonal_movement
     (setq _diagonal_movement (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _heuristic_cost_weight
     (setq _heuristic_cost_weight (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass astar_path_planner::PlanPathResponse
  :super ros::object
  :slots (_length_of_path _number_of_nodes_in_closed_set ))

(defmethod astar_path_planner::PlanPathResponse
  (:init
   (&key
    ((:length_of_path __length_of_path) 0)
    ((:number_of_nodes_in_closed_set __number_of_nodes_in_closed_set) 0)
    )
   (send-super :init)
   (setq _length_of_path (round __length_of_path))
   (setq _number_of_nodes_in_closed_set (round __number_of_nodes_in_closed_set))
   self)
  (:length_of_path
   (&optional __length_of_path)
   (if __length_of_path (setq _length_of_path __length_of_path)) _length_of_path)
  (:number_of_nodes_in_closed_set
   (&optional __number_of_nodes_in_closed_set)
   (if __number_of_nodes_in_closed_set (setq _number_of_nodes_in_closed_set __number_of_nodes_in_closed_set)) _number_of_nodes_in_closed_set)
  (:serialization-length
   ()
   (+
    ;; int32 _length_of_path
    4
    ;; int32 _number_of_nodes_in_closed_set
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _length_of_path
       (write-long _length_of_path s)
     ;; int32 _number_of_nodes_in_closed_set
       (write-long _number_of_nodes_in_closed_set s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _length_of_path
     (setq _length_of_path (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _number_of_nodes_in_closed_set
     (setq _number_of_nodes_in_closed_set (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass astar_path_planner::PlanPath
  :super ros::object
  :slots ())

(setf (get astar_path_planner::PlanPath :md5sum-) "64724dde033fbe8f19a82fd1a78b4bdc")
(setf (get astar_path_planner::PlanPath :datatype-) "astar_path_planner/PlanPath")
(setf (get astar_path_planner::PlanPath :request) astar_path_planner::PlanPathRequest)
(setf (get astar_path_planner::PlanPath :response) astar_path_planner::PlanPathResponse)

(defmethod astar_path_planner::PlanPathRequest
  (:response () (instance astar_path_planner::PlanPathResponse :init)))

(setf (get astar_path_planner::PlanPathRequest :md5sum-) "64724dde033fbe8f19a82fd1a78b4bdc")
(setf (get astar_path_planner::PlanPathRequest :datatype-) "astar_path_planner/PlanPathRequest")
(setf (get astar_path_planner::PlanPathRequest :definition-)
      "bool diagonal_movement
float64 heuristic_cost_weight
---
int32 length_of_path
int32 number_of_nodes_in_closed_set

")

(setf (get astar_path_planner::PlanPathResponse :md5sum-) "64724dde033fbe8f19a82fd1a78b4bdc")
(setf (get astar_path_planner::PlanPathResponse :datatype-) "astar_path_planner/PlanPathResponse")
(setf (get astar_path_planner::PlanPathResponse :definition-)
      "bool diagonal_movement
float64 heuristic_cost_weight
---
int32 length_of_path
int32 number_of_nodes_in_closed_set

")



(provide :astar_path_planner/PlanPath "64724dde033fbe8f19a82fd1a78b4bdc")


