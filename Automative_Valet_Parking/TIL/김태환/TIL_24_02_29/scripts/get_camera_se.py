#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

def Camera_callback(data):
    #os.system('clear')
    #rospy.loginfo('------hi------')
    #rospy.loginfo(data.data)
    np_arr = np.fromstring(data.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # cv2.imshow("Image window", img_bgr)
    # cv2.waitKey(1)

def listener():
    rospy.init_node('camera_img_se', anonymous=True)

    rospy.Subscriber("/camera_se", CompressedImage, Camera_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
