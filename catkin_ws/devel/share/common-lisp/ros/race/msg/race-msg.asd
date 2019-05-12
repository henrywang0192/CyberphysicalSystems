
(cl:in-package :asdf)

(defsystem "race-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "drive_param" :depends-on ("_package_drive_param"))
    (:file "_package_drive_param" :depends-on ("_package"))
    (:file "drive_values" :depends-on ("_package_drive_values"))
    (:file "_package_drive_values" :depends-on ("_package"))
    (:file "pid_input" :depends-on ("_package_pid_input"))
    (:file "_package_pid_input" :depends-on ("_package"))
    (:file "std_msgs" :depends-on ("_package_std_msgs"))
    (:file "_package_std_msgs" :depends-on ("_package"))
  ))