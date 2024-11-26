#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from std_msgs.msg import Int8, String
from enum import Enum
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from pynput import keyboard

# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

#TODO: (0) 필수 학습 지식
'''
# advanced_purepursuit 은 Pure Pursuit 알고리즘을 강화 한 예제입니다.
# 이전까지 사용한 Pure Pursuit 알고리즘은 고정된 전방주시거리(Look Forward Distance) 값을 사용하였습니다.
# 해당 예제에서는 전방주시거리(Look Forward Distance) 값을 주행 속도에 비례한 값으로 설정합니다.
# 이때 최소 최대 전방주시거리(Look Forward Distance) 를 설정합니다.
# 주행 속도에 비례한 값으로 변경 한 뒤 "self.lfd_gain" 을 변경 하여서 직접 제어기 성능을 튜닝 해보세요.
# 

'''

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #arg = rospy.myargv(argv=sys.argv)
        #local_path_name = arg[1]

        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/global_change_cmd", Int8, self.path_change_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber( 'Ego_topic' , EgoVehicleStatus , self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=10)
        self.rrt_start = rospy.Publisher('/start_rrt', String, queue_size=10) ## RRT 시작 알리는 Publisher
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_entrance = False
        self.is_inside = False
        self.is_start = True
        self.is_look_forward_point = False
        self.len_of_current_global_path = 0
        self.forward_point = Point()
        self.current_postion = Point()
        # self.status_msg = EgoVehicleStatus()

        # RRT 시작점
        self.is_parking_mode = False
        self.parking_start_x = 0.60
        self.parking_start_y = 1030.89

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = [30, 20, 20]
        self.number_of_current_path = 0

        self.pid = pidControl()
        self.vel_planning_dijkstra = velocityPlanning(self.target_velocity[0]/3.6, 0.7)
        self.vel_planning_entrance = velocityPlanning(self.target_velocity[1]/3.6, 0.7)
        self.vel_planning_inside = velocityPlanning(self.target_velocity[2]/3.6, 0.7)


        while True:
            if self.is_global_path == True:
                self.len_of_current_global_path = len(self.global_path.poses)
                self.velocity_list = self.vel_planning_dijkstra.curvedBaseVelocity(self.global_path, 50)
                # 초기 Global Path 기반 각 point에서 속도를 몇으로 달릴지 결정한 후 velocity_list에 속도 계획을 저장
                rospy.loginfo("Ready")
                self.send_gear_cmd(Gear.D.value)
                is_global_path = False
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            print(self.len_of_current_global_path)
            # path정보, imu + gps 정보, 차량 staus 정보 저장
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                # parking mode로 진입하면 break
                if self.is_parking_mode == True:
                    break
                if self.is_entrance==True and self.number_of_current_path==1 and not self.is_parking_mode:
                    self.lfd = 15
                    self.min_lfd = 10
                    self.max_lfd = 30
                    self.lfd_gain = 0.78
                    print("************************************************************************************")
                    print(len(self.global_path.poses))
                    self.velocity_list = self.vel_planning_entrance.curvedBaseVelocity(self.global_path, 80)
                    print(len(self.velocity_list))
                    self.is_entrance=False

                if self.is_inside==True and not self.is_parking_mode:
                    self.ctrl_cmd_msg.brake = 0.5
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    if  self.len_of_current_global_path==617:
                        self.lfd = 8
                        self.min_lfd = 5
                        self.max_lfd = 30
                        self.lfd_gain = 0.78
                        print("************************************************************************************")
                        print("주차장 내부 속도 계획 시작!")
                        print(len(self.global_path.poses))
                        self.velocity_list = self.vel_planning_inside.curvedBaseVelocity(self.global_path, 100)
                        print(len(self.velocity_list))
                        self.is_inside = False

                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                # print("Current Waypoint")
                # print(self.current_waypoint)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                steering = self.calc_pure_pursuit()

                if self.is_look_forward_point and not self.is_parking_mode:
                    self.ctrl_cmd_msg.steering = steering
                elif self.is_parking_mode:
                    break
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.brake = 0.5
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                # 속도 계획과 현재 차량 속도를 기준으로 PID 제어값 적용

                print('hihihi')
                # PID 제어 값 결과에 따라 양수면 accel에 output 입력, 음수면 break에 -output 입력
                if output > 0.0 and not self.is_parking_mode:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                elif self.is_parking_mode:
                    break
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
                
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  


    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

        dist = sqrt(pow(self.current_postion.x - self.parking_start_x, 2)+ pow(self.current_postion.y - self.parking_start_y, 2))
        if dist<2:
            self.ctrl_cmd_msg.accel = 0.0
            self.ctrl_cmd_msg.brake = 0.5
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            if self.is_parking_mode == False:
                self.rrt_start.publish("Start RRT Algorithm")
                self.is_parking_mode = True
            
    def send_gear_cmd(self, gear_mode):
        while(abs(self.status_msg.velocity.x) > 0.1):
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
        self.ctrl_cmd_pub.publish(cmd)

    def path_change_callback(self, msg):
        self.ch_num = msg.data
        while True:
            if self.is_global_path == True:
                if self.ch_num==1:
                    print("주차장 진입경로로 변경!")
                    self.is_global_path = False
                    self.is_entrance = True
                    break
                elif self.ch_num==2:
                    print("주차장 내부경로로 변경!")
                    self.is_global_path = False
                    self.is_inside = True
                    print("path_change_Callback!!!")
                    break
            else:
                    rospy.loginfo('Waiting global path data')
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 0.5
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg
        
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
        if len(self.global_path.poses) != self.len_of_current_global_path:
            self.number_of_current_path += 1
            self.len_of_current_global_path = len(self.global_path.poses)
            

    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = self.current_postion.x - pose.pose.position.x
            dy = self.current_postion.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
                #print(i)
        # print(currnet_waypoint)
        return currnet_waypoint

    def calc_pure_pursuit(self,):

        self.lfd = self.lfd_gain * self.status_msg.velocity.x
        self.lfd = max(self.min_lfd, min(self.max_lfd, self.lfd))  # 최소값과 최대값 사이의 값으로 조정
        # rospy.loginfo(self.lfd)  # 현재 lfd 값 로그로 출력

        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([[cos(self.vehicle_yaw),-sin(self.vehicle_yaw) ,0],
                                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),0],
                                [0,0,1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point = i.pose.position

            global_path_point = [path_point.x - vehicle_position.x, path_point.y - vehicle_position.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0] > 0:

                angle_to_point = atan2(local_path_point[1], local_path_point[0])
                if abs(angle_to_point) <= pi/2:  # 정면에 있는 경우에만
                    dis = sqrt(local_path_point[0] ** 2 + local_path_point[1] ** 2)

                    if dis >= self.lfd:
                        self.forward_point = local_path_point
                        self.is_look_forward_point = True
                        break

        theta = atan2(self.forward_point[1], self.forward_point[0])

        steering = atan2( ( 2 * self.vehicle_length * sin(theta)), sqrt(self.forward_point[0]**2 + self.forward_point[1]**2))
        print(steering)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #(5) PID 제어 생성
        
        # 종방향 제어를 위한 PID 제어기는 현재 속도와 목표 속도 간 차이를 측정하여 Accel/Brake 값을 결정 합니다.
        # 각 PID 제어를 위한 Gain 값은 "class pidContorl" 에 정의 되어 있습니다.
        # 각 PID Gain 값을 직접 튜닝하고 아래 수식을 채워 넣어 P I D 제어기를 완성하세요.

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * ((error - self.prev_error) / self.controlTime)

        output = p_control + self.i_control + d_control
        self.prev_error = error


        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            A = np.array(x_list)
            B = np.array(y_list)
            result = np.linalg.lstsq(A, B, rcond=None)

            if len(result[0]) == 0:
                continue

            r = sqrt(result[0][0] ** 2 + result[0][1] ** 2)  # 곡률 반경 계산
            gravityAcc = 9.8
            v_max = sqrt(r * self.road_friction * gravityAcc)  # 최대 속도 계획


            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(3)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(3)
        return out_vel_plan

if __name__ == '__main__':
    while(True):
        pause_input = int(input('Enter 1 to continue: '))

        if pause_input == 1:
            break

    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
