; Auto-generated. Do not edit!


(cl:in-package yoloswag-srv)


;//! \htmlinclude RecordAudio-request.msg.html

(cl:defclass <RecordAudio-request> (roslisp-msg-protocol:ros-message)
  ((d
    :reader d
    :initarg :d
    :type cl:integer
    :initform 0)
   (cmds
    :reader cmds
    :initarg :cmds
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
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

(cl:ensure-generic-function 'cmds-val :lambda-list '(m))
(cl:defmethod cmds-val ((m <RecordAudio-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yoloswag-srv:cmds-val is deprecated.  Use yoloswag-srv:cmds instead.")
  (cmds m))
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cmds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'cmds))
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cmds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cmds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
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
  "19618e2dbb865e4c55eb07772ccdfd38")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecordAudio-request)))
  "Returns md5sum for a message object of type 'RecordAudio-request"
  "19618e2dbb865e4c55eb07772ccdfd38")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecordAudio-request>)))
  "Returns full string definition for message of type '<RecordAudio-request>"
  (cl:format cl:nil "int64 d~%string[] cmds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecordAudio-request)))
  "Returns full string definition for message of type 'RecordAudio-request"
  (cl:format cl:nil "int64 d~%string[] cmds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecordAudio-request>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cmds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecordAudio-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RecordAudio-request
    (cl:cons ':d (d msg))
    (cl:cons ':cmds (cmds msg))
))
;//! \htmlinclude RecordAudio-response.msg.html

(cl:defclass <RecordAudio-response> (roslisp-msg-protocol:ros-message)
  ((hypothesis
    :reader hypothesis
    :initarg :hypothesis
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
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

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <RecordAudio-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yoloswag-srv:confidence-val is deprecated.  Use yoloswag-srv:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecordAudio-response>) ostream)
  "Serializes a message object of type '<RecordAudio-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'hypothesis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'hypothesis))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecordAudio-response>) istream)
  "Deserializes a message object of type '<RecordAudio-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'hypothesis) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'hypothesis)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
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
  "19618e2dbb865e4c55eb07772ccdfd38")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecordAudio-response)))
  "Returns md5sum for a message object of type 'RecordAudio-response"
  "19618e2dbb865e4c55eb07772ccdfd38")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecordAudio-response>)))
  "Returns full string definition for message of type '<RecordAudio-response>"
  (cl:format cl:nil "string[] hypothesis~%float32 confidence~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecordAudio-response)))
  "Returns full string definition for message of type 'RecordAudio-response"
  (cl:format cl:nil "string[] hypothesis~%float32 confidence~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecordAudio-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'hypothesis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecordAudio-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RecordAudio-response
    (cl:cons ':hypothesis (hypothesis msg))
    (cl:cons ':confidence (confidence msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RecordAudio)))
  'RecordAudio-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RecordAudio)))
  'RecordAudio-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecordAudio)))
  "Returns string type for a service object of type '<RecordAudio>"
  "yoloswag/RecordAudio")