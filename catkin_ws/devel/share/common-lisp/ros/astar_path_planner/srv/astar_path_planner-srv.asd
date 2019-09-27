
(cl:in-package :asdf)

(defsystem "astar_path_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PlanPath" :depends-on ("_package_PlanPath"))
    (:file "_package_PlanPath" :depends-on ("_package"))
  ))