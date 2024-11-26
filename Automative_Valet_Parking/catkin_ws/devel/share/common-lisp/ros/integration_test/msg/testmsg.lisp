; Auto-generated. Do not edit!


(cl:in-package integration_test-msg)


;//! \htmlinclude testmsg.msg.html

(cl:defclass <testmsg> (roslisp-msg-protocol:ros-message)
  ((sequenceNum
    :reader sequenceNum
    :initarg :sequenceNum
    :type cl:fixnum
    :initform 0))
)

(cl:defclass testmsg (<testmsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <testmsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'testmsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name integration_test-msg:<testmsg> is deprecated: use integration_test-msg:testmsg instead.")))

(cl:ensure-generic-function 'sequenceNum-val :lambda-list '(m))
(cl:defmethod sequenceNum-val ((m <testmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader integration_test-msg:sequenceNum-val is deprecated.  Use integration_test-msg:sequenceNum instead.")
  (sequenceNum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <testmsg>) ostream)
  "Serializes a message object of type '<testmsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequenceNum)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <testmsg>) istream)
  "Deserializes a message object of type '<testmsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequenceNum)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<testmsg>)))
  "Returns string type for a message object of type '<testmsg>"
  "integration_test/testmsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'testmsg)))
  "Returns string type for a message object of type 'testmsg"
  "integration_test/testmsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<testmsg>)))
  "Returns md5sum for a message object of type '<testmsg>"
  "96f4abd1a5050e1e1afc651209403d91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'testmsg)))
  "Returns md5sum for a message object of type 'testmsg"
  "96f4abd1a5050e1e1afc651209403d91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<testmsg>)))
  "Returns full string definition for message of type '<testmsg>"
  (cl:format cl:nil "uint8 sequenceNum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'testmsg)))
  "Returns full string definition for message of type 'testmsg"
  (cl:format cl:nil "uint8 sequenceNum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <testmsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <testmsg>))
  "Converts a ROS message object to a list"
  (cl:list 'testmsg
    (cl:cons ':sequenceNum (sequenceNum msg))
))
