
(cl:in-package :asdf)

(defsystem "kinect_anywhere-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointPositionAndState" :depends-on ("_package_JointPositionAndState"))
    (:file "_package_JointPositionAndState" :depends-on ("_package"))
    (:file "BodyFrame" :depends-on ("_package_BodyFrame"))
    (:file "_package_BodyFrame" :depends-on ("_package"))
    (:file "Body" :depends-on ("_package_Body"))
    (:file "_package_Body" :depends-on ("_package"))
  ))