;; Auto-generated. Do not edit!


(when (boundp 'integration_test::testmsg)
  (if (not (find-package "INTEGRATION_TEST"))
    (make-package "INTEGRATION_TEST"))
  (shadow 'testmsg (find-package "INTEGRATION_TEST")))
(unless (find-package "INTEGRATION_TEST::TESTMSG")
  (make-package "INTEGRATION_TEST::TESTMSG"))

(in-package "ROS")
;;//! \htmlinclude testmsg.msg.html


(defclass integration_test::testmsg
  :super ros::object
  :slots (_sequenceNum ))

(defmethod integration_test::testmsg
  (:init
   (&key
    ((:sequenceNum __sequenceNum) 0)
    )
   (send-super :init)
   (setq _sequenceNum (round __sequenceNum))
   self)
  (:sequenceNum
   (&optional __sequenceNum)
   (if __sequenceNum (setq _sequenceNum __sequenceNum)) _sequenceNum)
  (:serialization-length
   ()
   (+
    ;; uint8 _sequenceNum
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _sequenceNum
       (write-byte _sequenceNum s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _sequenceNum
     (setq _sequenceNum (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get integration_test::testmsg :md5sum-) "96f4abd1a5050e1e1afc651209403d91")
(setf (get integration_test::testmsg :datatype-) "integration_test/testmsg")
(setf (get integration_test::testmsg :definition-)
      "uint8 sequenceNum

")



(provide :integration_test/testmsg "96f4abd1a5050e1e1afc651209403d91")


