; Auto-generated. Do not edit!


(cl:in-package yoloswag-srv)


;//! \htmlinclude RecordAudio-request.msg.html

(cl:defclass <RecordAudio-request> (roslisp-msg-protocol:ros-message)
  ((d
    :reader d
    :initarg :d
    :type cl:integer
    :initform 0))
)

(cl:defclass RecordAudio-request (<RecordAudio-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecordAudio-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecordAudio-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yoloswag-srv:<RecordAudio-request> is deprecated: use yoloswag-srv:RecordAudio-request instead.")))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <RecordAudio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yoloswag-srv:d-val is deprecated.  Use yoloswag-srv:d instead.")
  (d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecordAudio-request>) ostream)
  "Serializes a message object of type '<RecordAudio-request>"
  (cl:let* ((signed (cl:slot-value msg 'd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecordAudio-request>) istream)
  "Deserializes a message object of type '<RecordAudio-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'd) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecordAudio-request>)))
  "Returns string type for a service object of type '<RecordAudio-request>"
  "yoloswag/RecordAudioRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordAudio-request)))
  "Returns string type for a service object of type 'RecordAudio-request"
  "yoloswag/RecordAudioRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecordAudio-request>)))
  "Returns md5sum for a message object of type '<RecordAudio-request>"
  "bed649305b6ee19b4403c3008fcf992f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecordAudio-request)))
  "Returns md5sum for a message object of type 'RecordAudio-request"
  "bed649305b6ee19b4403c3008fcf992f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecordAudio-request>)))
  "Returns full string definition for message of type '<RecordAudio-request>"
  (cl:format cl:nil "int64 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecordAudio-request)))
  "Returns full string definition for message of type 'RecordAudio-request"
  (cl:format cl:nil "int64 d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecordAudio-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecordAudio-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RecordAudio-request
    (cl:cons ':d (d msg))
))
;//! \htmlinclude RecordAudio-response.msg.html

(cl:defclass <RecordAudio-response> (roslisp-msg-protocol:ros-message)
  ((hypothesis
    :reader hypothesis
    :initarg :hypothesis
    :type cl:string
    :initform ""))
)

(cl:defclass RecordAudio-response (<RecordAudio-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecordAudio-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecordAudio-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yoloswag-srv:<RecordAudio-response> is deprecated: use yoloswag-srv:RecordAudio-response instead.")))

(cl:ensure-generic-function 'hypothesis-val :lambda-list '(m))
(cl:defmethod hypothesis-val ((m <RecordAudio-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yoloswag-srv:hypothesis-val is deprecated.  Use yoloswag-srv:hypothesis instead.")
  (hypothesis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecordAudio-response>) ostream)
  "Serializes a message object of type '<RecordAudio-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hypothesis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hypothesis))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecordAudio-response>) istream)
  "Deserializes a message object of type '<RecordAudio-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hypothesis) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hypothesis) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecordAudio-response>)))
  "Returns string type for a service object of type '<RecordAudio-response>"
  "yoloswag/RecordAudioResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordAudio-response)))
  "Returns string type for a service object of type 'RecordAudio-response"
  "yoloswag/RecordAudioResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecordAudio-response>)))
  "Returns md5sum for a message object of type '<RecordAudio-response>"
  "bed649305b6ee19b4403c3008fcf992f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecordAudio-response)))
  "Returns md5sum for a message object of type 'RecordAudio-response"
  "bed649305b6ee19b4403c3008fcf992f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecordAudio-response>)))
  "Returns full string definition for message of type '<RecordAudio-response>"
  (cl:format cl:nil "string hypothesis~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecordAudio-response)))
  "Returns full string definition for message of type 'RecordAudio-response"
  (cl:format cl:nil "string hypothesis~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecordAudio-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'hypothesis))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecordAudio-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RecordAudio-response
    (cl:cons ':hypothesis (hypothesis msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RecordAudio)))
  'RecordAudio-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RecordAudio)))
  'RecordAudio-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordAudio)))
  "Returns string type for a service object of type '<RecordAudio>"
  "yoloswag/RecordAudio")