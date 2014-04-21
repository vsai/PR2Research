
(cl:in-package :asdf)

(defsystem "yoloswag-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RecordAudio" :depends-on ("_package_RecordAudio"))
    (:file "_package_RecordAudio" :depends-on ("_package"))
  ))