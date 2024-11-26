#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import GetTrafficLightStatus

def traffic_light_callback(data):
    os.system('clear')
    rospy.loginfo('-------------------- Traffic Light Vehicle -------------------------')
    rospy.loginfo("Traffic Light Idx    : {}".format(data.trafficLightIndex))
    rospy.loginfo("Traffic Light Status : {}".format(data.trafficLightStatus))
    rospy.loginfo("Traffic Light Type   : {}".format(data.trafficLightType))

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('traffic_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, traffic_light_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
