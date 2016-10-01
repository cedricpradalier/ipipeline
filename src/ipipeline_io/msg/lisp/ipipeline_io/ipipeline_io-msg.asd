
(in-package :asdf)

(defsystem "ipipeline_io-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "ProbeStatus" :depends-on ("_package"))
    (:file "_package_ProbeStatus" :depends-on ("_package"))
    (:file "ProbeCommand" :depends-on ("_package"))
    (:file "_package_ProbeCommand" :depends-on ("_package"))
    ))
