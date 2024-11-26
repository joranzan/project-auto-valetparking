#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import os
from math import sqrt
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from enum import Enum

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class v_loc():
    def __init__(self):
        rospy.init_node('vehicle_location', anonymous=True)
        
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        
        # 초기화
        self.is_loc=False
        self.ego_status = EgoVehicleStatus()

        # 메시지 발행 속도 설정
        self.rate = rospy.Rate(10)
        
        
        while not rospy.is_shutdown():
            if self.is_loc==True:
                
                os.system('clear')
                print(self.ego_status)
                print("Now Speed", self.ego_status.velocity.x*3.6)

                self.rate.sleep()

    def ego_callback(self, data):
        self.ego_status = data
        self.is_loc=True

if __name__ == '__main__':
    try:
        v_loc = v_loc()
    except rospy.ROSInterruptException:
        pass