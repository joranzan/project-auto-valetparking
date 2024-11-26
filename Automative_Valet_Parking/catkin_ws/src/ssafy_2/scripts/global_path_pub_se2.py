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
    def __init__(self, pkg_name = 'ssafy_2'):
        rospy.init_node('global_path_pub', anonymous = True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)


        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.path_name = "parkinglot_driving_se2"

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        # 주차장 진입 경로 읽어오기
        full_path_enter = pkg_path + "/path/" +self.path_name + ".txt"
        self.f_enter = open(full_path_enter, 'r')
        lines_enter = self.f_enter.readlines()

        for line in lines_enter :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.global_path_msg.poses.append(read_pose)     
        self.f_enter.close()


        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (4) Global Path 정보 Publish
            
            self.global_path_pub.publish(self.global_path_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass
