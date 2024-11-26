#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import os
import math
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

class a_parking():
    
    def __init__(self):
        rospy.init_node('auto_parking', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)

        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        
        # 초기화
        self.is_loc=False
        
        self.ego_status = EgoVehicleStatus()
        wheelbase = 2.414462773659777
        # wheelbase = 2.414462773659777
        
        target_position = Vector3(0, 0, 0) # 주차선 앞 기준
        target_line_dot1 = Vector3(2.18, 1020.99, 0) # 후진 도착 위치1
        target_line_dot2 = Vector3(4.43279695511, 1019.83178711, -0.875219464302) # 후진 도착 위치2
        target_angle = 59.79750583426879 #61.4259338379
        target_1 = Vector3(3.27, 1025.04, -1.15)
        target_2 = Vector3(5.18, 1023.99, -1.15)
        target_3 = Vector3(2.69, 1019.72, -1.22)
        target_4 = Vector3(0.79, 1020.78, -1.14)
        target_position.x = (target_1.x + target_2.x)/2
        target_position.y = (target_1.y + target_2.y)/2
        
        #self.target_position = Vector3(-4.85, 1050.03, 0)
        #target_1 = Vector3(-1.00, 1049.20, -0.80)
        #target_2 = Vector3(-2.11, 1047.22, -0.82)
        #target_3 = Vector3(-6.27, 1049.52, -0.77)
        #target_4 = Vector3(-5.14, 1051.50, -0.74)
    
        start_position = Vector3(0, 0, 0)
        start_angle = None
        std1, std2 = None, None
        steering_value = None
        circle_angle = None
        distance = None
        line_a = (target_line_dot2.y - target_line_dot1.y)/(target_line_dot2.x - target_line_dot1.x)
        line_b = -1
        line_c = target_line_dot1.x*(target_line_dot1.y - target_line_dot2.y)/(target_line_dot2.x - target_line_dot1.x) + target_line_dot1.y
        
        # 메시지 발행 속도 설정
        self.rate = rospy.Rate(10)
        
        start_time = time.time()
        while (time.time() - start_time) < 1:
            print("test")
        
        start_angle = self.ego_status.heading
        start_position = self.ego_status.position
        
        distance = math.sqrt((start_position.x - target_position.x)**2 + (start_position.y - target_position.y)**2)
        
        std1 = target_angle - start_angle
        std2 = 360 - abs(target_angle - start_angle)
        if(std1 >= 0):
            std2*=(-1)
            
        if(abs(std1) < abs(std2)):
            if(std1 >= 0):
                steering_value = -1
            else:
                steering_value = 1
            circle_angle = math.radians(abs(std1)/2)
        else:
            if(std2 >= 0):
                steering_value = -1
            else:
                steering_value = 1
            circle_angle = math.radians(abs(std2)/2)
            
        steering_value *= math.pi*36.25/180
        steering_value *= math.degrees(math.asin(2*wheelbase*math.sin(circle_angle)/distance))/36.25

        self.send_gear_cmd(Gear.R.value)
        
        for _ in range(5):
            self.send_ctrl_cmd(steering_value, 1)
            self.rate.sleep()
        
        while(abs(self.ego_status.heading - target_angle) > 2): # 3.5
            os.system('clear')
            print("steering_value : ", steering_value)
            print(abs(self.ego_status.heading - target_angle))
            
            if(abs(self.ego_status.velocity.x*3.6) > 1.5):
                self.send_ctrl_cmd2(0.05, steering_value)
            else:
                self.send_ctrl_cmd(steering_value, 1)
            self.rate.sleep()
        
        while(1):
            dis = abs(self.ego_status.position.x*line_a + self.ego_status.position.y*line_b + line_c)/math.sqrt(line_a**2 + line_b**2)
            os.system('clear')
            print(dis)
            if(dis < 0.2):
                break
            
            self.send_ctrl_cmd(0, 4)
            self.rate.sleep()
        
        self.send_gear_cmd(Gear.P.value)

    def ego_callback(self, data):
        self.ego_status = data
        self.is_loc=True

    def send_gear_cmd(self, gear_mode):
        while(abs(self.ego_status.velocity.x) > 0.1):
            self.send_ctrl_cmd(0, 0)
            self.rate.sleep()
        
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)

    def send_ctrl_cmd(self, steering, velocity):
        cmd = CtrlCmd()
        if(velocity > 0):
            cmd.longlCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.cmd_pub.publish(cmd)
    
    def send_ctrl_cmd2(self, brake, steering):
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.brake = brake
        cmd.steering = steering
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        a_d = a_parking()
    except rospy.ROSInterruptException:
        pass
