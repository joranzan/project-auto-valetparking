#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Int8, String

# global_path_pub 은 txt 파일로 저장한 Path 데이터를 global Path (전역경로) 로 읽어오는 예제입니다.
# 만들어진 global Path(전역경로) 는 Local Path (지역경로) 를 만드는데 사용 된다.

# 노드 실행 순서 
# 1. Global Path publisher 선언 및 Global Path 변수 생성 
# 2. 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
# 3. 읽어 온 경로 데이터를 Global Path 변수에 넣기
# 4. Global Path 정보 Publish


class global_path_pub :
    def __init__(self, pkg_name = 'rrt_drive1'):
        rospy.init_node('global_path_pub', anonymous = True)

        # Subscriber
        rospy.Subscriber("/global_change_cmd", Int8, self.path_change_callback)
        rospy.Subscriber("/dijkstra_path", Path, self.dijkstra_callback)
        rospy.Subscriber("/start_rrt", String, self.start_rrt_callback)
        rospy.Subscriber("/done", String, self.rrt_path_callback)
        # Global Path 데이터를 Publish 하는 변수와 메세지를 담고있는 변수를 선언한다.
        # 이때 Global Path 는 map 좌표계를 기준으로 생성한다.
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.dijkstra_flag = rospy.Publisher("/Dijkstra_Flag", String, queue_size=10)
        self.path_all_done_start_parking = rospy.Publisher("/ready_to_park", String, queue_size = 10)
        
        self.is_dijkstra = True


        self.is_parking_mode = False
        self.is_parking_path_completed = False
        self.is_good_to_publish = False

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        # 다익스트라 경로
        self.global_path_msg_dijkstra = Path()
        self.global_path_msg_dijkstra.header.frame_id = "/map"
        # 진입 경로
        self.global_path_msg_enter = Path()
        self.global_path_msg_enter.header.frame_id = "/map"
        # 내부 경로
        self.global_path_msg_inside = Path()
        self.global_path_msg_inside.header.frame_id = "/map"

        self.global_path_msg_rrt = Path()
        self.global_path_msg_rrt.header.frame_id = "/map"

        self.path_name = ["scen1_2", "parkinglot_driving_test_tiny020", "ssafy_test_path_star"]
        self.recieved_cmd = 0
        self.load = 0

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        # 주차장 진입 경로 읽어오기
        full_path_enter = pkg_path + "/path/" +self.path_name[0] + ".txt"
        self.f_enter = open(full_path_enter, 'r')
        lines_enter = self.f_enter.readlines()

        for line in lines_enter :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.global_path_msg_enter.poses.append(read_pose)     
        self.f_enter.close()

        # 주차장 내부 탐색 경로 읽어오기
        full_path_inside = pkg_path + "/path/" +self.path_name[1] + ".txt"
        self.f_inside = open(full_path_inside, 'r')     
        lines_inside = self.f_inside.readlines()
        
        for line in lines_inside :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.global_path_msg_inside.poses.append(read_pose)     
        self.f_inside.close()

        self.full_path_rrt = pkg_path + "/path/" + self.path_name[2] + ".txt"
        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (4) Global Path 정보 Publish
            if self.is_parking_mode == False:
                if self.load == 0 :
                    self.global_path_pub.publish(self.global_path_msg_dijkstra)
                elif self.load == 1:
                    self.global_path_pub.publish(self.global_path_msg_enter)
                elif self.load == 2:
                    self.global_path_pub.publish(self.global_path_msg_inside)
                else:
                    # Global Path 메세지 를 전송하는 publisher 를 만든다.
                    self.global_path_pub.publish(self.global_path_msg)
                    
            elif self.is_parking_mode == True:
                if self.is_parking_path_completed == False:
                    print("Waiting for RRT Path........")
                else:
                    if self.is_good_to_publish ==True:
                        print("RRT Path all done")
                        print(self.global_path_msg_rrt.poses)
                        self.path_all_done_start_parking.publish("Completed")
                        self.is_good_to_publish = False
                    else:
                        self.global_path_pub.publish(self.global_path_msg_rrt)
            
            rate.sleep()

    def dijkstra_callback(self, msg):
        print("Hi Dijkstra Path")
        # print(msg)
        self.global_path_msg_dijkstra = msg
        self.is_dijkstra = True
        self.dijkstra_flag.publish("Subscribed Completely")

    def path_change_callback(self, msg):
        self.recieved_cmd = msg.data
        if self.recieved_cmd == 0:
            self.load = 0
            
        elif self.recieved_cmd == 1:
            self.load = 1
            rospy.loginfo(self.load)
        elif self.recieved_cmd == 2:
            self.load = 2
            rospy.loginfo(self.load)


    def start_rrt_callback(self, msg):
        self.is_parking_mode = True
        
    
    def rrt_path_callback(self, msg):
        if msg.data == 'done':
            self.path_read()
            self.is_parking_path_completed = True

    
    def path_read(self):
        
        self.f = open(self.full_path_rrt, 'r')
        lines = self.f.readlines()
        
        for line in lines :
            # line : 140.195312555   1380.04650697   0.0
            # split : 문자열 배열로 변환
            # tmp : ['140.195312555', '1380.04650697', '0.0']
            # type이 string 이므로 float으로 변환
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            # read_pose.pose.orientation.w = float(tmp[3])
            # read_pose.pose.orientation.w = 1
            self.global_path_msg_rrt.poses.append(read_pose)        
        self.f.close()
        self.global_path_pub.publish(self.global_path_msg_rrt)
        self.is_good_to_publish = True


if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass
