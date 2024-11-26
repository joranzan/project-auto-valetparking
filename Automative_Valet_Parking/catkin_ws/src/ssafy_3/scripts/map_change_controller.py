#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,pi,sqrt,pow,atan2

from std_msgs.msg import Int8, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class Controller :
    def __init__(self):
        rospy.init_node('Controller_cmd', anonymous=True)
        
        # Publisher
        self.ctrl_cmd_pub = rospy.Publisher('/global_change_cmd', Int8, queue_size=1)  # Global 경로 변경 명령어
        self.path_changed = rospy.Publisher("/path_changed", String, queue_size=10)

        # Subscriber
        rospy.Subscriber("/odom", Odometry, self.odom_callback)


        self.cmd_num = Int8()
        # 주차장 진입하는 경로로 변경 기준 좌표
        self.parkingLot_x1 = -30       # -20.5
        self.parkingLot_y1 = 1012      # 1035.3

        # 주차장 내부 경로로 변경 기준 좌표
        self.parkingLot_x2 = 20.0971146169       # 16.24
        self.parkingLot_y2 = 1059.72875795     # 1061.68

        self.is_odom = False
        self.is_dijkstra = False
        self.arrivedParkinglot = False
        self.in_ParkingLot = False
        self.current_position=Point()
        self.already_published_1 = False
        self.already_published_2 = False

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_odom == True:  
                # 주차장 근처 모드        
                if self.arrivedParkinglot==True:          
                    # parking_lot_entrance_path로 변경하라는 cmd publish
                    if  self.in_ParkingLot==False:
                        if self.already_published_1 ==False :
                            print("cmd 1 : change path to parking_lot_entrance_path!")
                            self.cmd_num = 1
                            self.ctrl_cmd_pub.publish(self.cmd_num)
                            self.already_published_1 = True
                    else:
                    # parking_lot_inside_path로 변경하라는 cmd publish
                        if self.already_published_2==False:
                            print("cmd 2 : change path to parking_lot_inside_path!")
                            self.cmd_num = 2
                            self.ctrl_cmd_pub.publish(self.cmd_num)
                            self.already_published_2 = True
                else:
                    if self.is_dijkstra==True:
                        print("cmd 0 : maintain Dijkstra Path from TakeOff Point")
                        self.cmd_num = 0
                    else :
                        print("Waiting for Dijkstra Path......")

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y

        # 주차장 근처 좌표까지 도달했는지 여부 판별
        if self.arrivedParkinglot == False :
            dist = sqrt(pow(self.current_position.x - self.parkingLot_x1, 2) + pow(self.current_position.y - self.parkingLot_y1, 2))
            if dist < 4 :
                self.path_changed.publish("Changed")
                self.arrivedParkinglot = True
        # 주차장 근처 좌표까지 도달했었다면 주차장 내부 진입 했는지 여부 판별
        else:
            dist = sqrt(pow(self.current_position.x - self.parkingLot_x2, 2) + pow(self.current_position.y - self.parkingLot_y2, 2))
            if dist < 3 :
                self.path_changed.publish("Changed")
                self.in_ParkingLot = True

if __name__ == '__main__':
    try:
        test_track=Controller()
    except rospy.ROSInterruptException:
        pass
