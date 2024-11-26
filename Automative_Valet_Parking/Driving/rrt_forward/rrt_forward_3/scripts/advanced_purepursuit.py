#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
from std_msgs.msg import Int8
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

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
class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        '''
        #TODO: ros Launch File <arg> Tag 
        # ros launch 파일 에는 여러 태그 를 사용 할 수 있지만 
        # 그중 <arg> 태그를 사용하여 변수를 정의 할 수 있습니다.
        # 3 장 에서는 사용하는 Path 정보와 Object 각 예제 별로 다르기 때문에
        # launch 파일의 <arg> 태그를 사용하여 예제에 맞게 변수를 설정합니다.
        
        '''
        #arg = rospy.myargv(argv=sys.argv)
        #local_path_name = arg[1]

        rospy.Subscriber("/lattice_path", Path, self.path_callback)

        #TODO: (1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/global_change_cmd", Int8, self.path_change_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber( 'Ego_topic' , EgoVehicleStatus , self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=10)

        

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_entrance = False
        self.is_inside = False
        self.is_look_forward_point = False
        self.len_of_global_path = 0
        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = [30, 20, 10]

        self.pid = pidControl()
        self.vel_planning_dijkstra = velocityPlanning(self.target_velocity[0]/3.6, 0.7)
        self.vel_planning_entrance = velocityPlanning(self.target_velocity[1]/3.6, 0.7)
        self.vel_planning_inside = velocityPlanning(self.target_velocity[2]/3.6, 0.7)

        while True:
            if self.is_global_path == True:
                self.len_of_global_path = len(self.global_path.poses)
                self.velocity_list = self.vel_planning_dijkstra.curvedBaseVelocity(self.global_path, 50)
                # 초기 Global Path 기반 각 point에서 속도를 몇으로 달릴지 결정한 후 velocity_list에 속도 계획을 저장
                rospy.loginfo("Ready")
                is_global_path = False
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            
            '''s
            필요한 추가 제어
            1. 특정 위치 도달 시 주차장 정보 Subscribe -> 주차장에 설치된 정보 전송 Publisher를 Subscribe하는 개념
            2. 주차장 Map에 대한 Global 경로 재 계획 -> 주차장 내 순회하는 루트로 변환
            3. 주차장 정보에서 내 위치 기반 인접한 주차장 Global Map상 Node로 이동
            4. Node로 이동 후 특정 Node에 도달할 때 마다 인접 주차공간 Node가 비어있는지 LiDAR를 통해 확인
                4-1. 인접 주차공간 Node를 Subscribe한 주차장 정보
            '''


            # path정보, imu + gps 정보, 차량 staus 정보 저장
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                if self.is_entrance==True and self.len_of_global_path != len(self.global_path.poses):
                    self.lfd = 15
                    self.min_lfd = 10
                    self.max_lfd = 30
                    self.lfd_gain = 0.78
                    print("************************************************************************************")
                    print(len(self.global_path.poses))
                    self.velocity_list = self.vel_planning_entrance.curvedBaseVelocity(self.global_path, 80)
                    print(len(self.velocity_list))
                    self.is_entrance=False
                    self.len_of_global_path = len(self.global_path.poses)

                if self.is_inside==True and self.len_of_global_path != len(self.global_path.poses):
                    self.lfd = 8
                    self.min_lfd = 0
                    self.max_lfd = 30
                    self.lfd_gain = 0.78
                    print("************************************************************************************")
                    print(len(self.global_path.poses))
                    self.velocity_list = self.vel_planning_inside.curvedBaseVelocity(self.global_path, 50)
                    print(len(self.velocity_list))
                    self.is_inside = False
                    self.len_of_global_path = len(self.global_path.poses)
                # print("Find Current Waypoint")
                #print(self.velocity_list)
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                # 매 주기마다 현재 차량의 위치와 가장 가까운 global_path의 point를 cur_point에 저장
                # print(self.current_waypoint)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                # 현재 위치에 해당하는 velocity_list의 속도 값 불러오기

                steering = self.calc_pure_pursuit()
                # 속도 계획과 다음 경로를 기반으로 pure_pursuit 수행
                if self.is_look_forward_point :
                    # 다음 갈 지점이 있다면 steering 값 저장
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    # self.ctrl_cmd_msg.brake = 0.5
                    # self.ctrl_cmd_msg.steering = 0.0
                    # self.ctrl_cmd_msg.accel = 0.0
                    # self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                # 속도 계획과 현재 차량 속도를 기준으로 PID 제어값 적용

                
                # PID 제어 값 결과에 따라 양수면 accel에 output 입력, 음수면 break에 -output 입력
                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # (8) 제어입력 메세지 Publish
                # print(self.ctrl_cmd_msg)
                # accel, break, steering 값을 Publish하여 시뮬레이터 차량에 전달
                # print(self.ctrl_cmd_msg)
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
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = self.current_postion.x - pose.pose.position.x
            dy = self.current_postion.y - pose.pose.position.y
            # print("Position List Position List Position List Position ListPosition List Position List")
            # print(ego_status.position.x)
            # print(self.current_postion.x)
            # print(ego_status.position.y)
            # print(self.current_postion.y)
            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
                #print(i)
        # print(currnet_waypoint)
        return currnet_waypoint

    def calc_pure_pursuit(self,):

        #(2) 속도 비례 Look Ahead Distance 값 설정
        '''
        # 차량 속도에 비례하여 전방주시거리(Look Forward Distance) 가 변하는 수식을 구현 합니다.
        # 이때 'self.lfd' 값은 최소와 최대 값을 넘어서는 안됩니다.
        # "self.min_lfd","self.max_lfd", "self.lfd_gain" 을 미리 정의합니다.
        # 최소 최대 전방주시거리(Look Forward Distance) 값과 속도에 비례한 lfd_gain 값을 직접 변경해 볼 수 있습니다.
        # 초기 정의한 변수 들의 값을 변경하며 속도에 비례해서 전방주시거리 가 변하는 advanced_purepursuit 예제를 완성하세요.
        # 
        self.lfd = 

        rospy.loginfo(self.lfd)

        '''
        self.lfd = self.lfd_gain * self.status_msg.velocity.x
        self.lfd = max(self.min_lfd, min(self.max_lfd, self.lfd))  # 최소값과 최대값 사이의 값으로 조정
        # rospy.loginfo(self.lfd)  # 현재 lfd 값 로그로 출력

        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        # (3) 좌표 변환 행렬 생성
        
        # Pure Pursuit 알고리즘을 실행 하기 위해서 차량 기준의 좌표계가 필요합니다.
        # Path 데이터를 현재 차량 기준 좌표계로 좌표 변환이 필요합니다.
        # 좌표 변환을 위한 좌표 변환 행렬을 작성합니다.
        # Path 데이터를 차량 기준 좌표 계로 변환 후 Pure Pursuit 알고리즘 중 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 찾습니다.
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 이용하여 조향 각도를 계산하게 됩니다.
        # 좌표 변환 행렬을 이용해 Path 데이터를 차량 기준 좌표 계로 바꾸는 반복 문을 작성 한 뒤
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 계산하는 로직을 작성 하세요.

        trans_matrix = np.array([[cos(self.vehicle_yaw),-sin(self.vehicle_yaw) ,0],
                                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),0],
                                [0,0,1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)
        # print(self.path.poses)
        for num,i in enumerate(self.path.poses) :
            path_point = i.pose.position

            global_path_point = [path_point.x - vehicle_position.x, path_point.y - vehicle_position.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    
            #print("g :", global_path_point)
            #print("l :", local_path_point)
            # if local_path_point[0]>0 :
            #     dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
            #     if dis >= self.lfd :
            #         self.forward_point = local_path_point
            #         self.is_look_forward_point = True
            #         break
            if local_path_point[0] > 0:
                # 각도를 계산하여 정면에 있는지 확인합니다.
                #print("local_path_point: {}, {}".format(local_path_point[0], local_path_point[1]))

                angle_to_point = atan2(local_path_point[1], local_path_point[0])
                if abs(angle_to_point) <= pi/2:  # 정면에 있는 경우에만
                    dis = sqrt(local_path_point[0] ** 2 + local_path_point[1] ** 2)
                    #print("************************************************")
                    #print(dis)
                    #print(self.lfd)
                    #self.is_look_forward_point = True
                    if dis >= self.lfd:
                        self.forward_point = local_path_point
                        self.is_look_forward_point = True
                        break

        
        
        # (4) Steering 각도 계산
        
        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        theta = atan2(self.forward_point[1], self.forward_point[0])

        # # 각도 범위를 -π에서 π까지로 조정합니다.
        # if theta < -pi/2:
        #     theta += pi
        # elif theta > pi/2:
        #     theta -= pi

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
        # print("Global Path Num : ")
        # print(len(gloabl_path.poses))
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

            # (6) 도로의 곡률 계산
            
            # 도로의 곡률 반경을 계산하기 위한 수식입니다.
            # Path 데이터의 좌표를 이용해서 곡선의 곡률을 구하기 위한 수식을 작성합니다.
            # 원의 좌표를 구하는 행렬 계산식, 최소 자승법을 이용하는 방식 등 곡률 반지름을 구하기 위한 식을 적용 합니다.
            # 적용한 수식을 통해 곡률 반지름 "r" 을 계산합니다.

            A = np.array(x_list)
            B = np.array(y_list)
            result = np.linalg.lstsq(A, B, rcond=None)

            if len(result[0]) == 0:
                continue

            r = sqrt(result[0][0] ** 2 + result[0][1] ** 2)  # 곡률 반경 계산
            gravityAcc = 9.8

            

            # (7) 곡률 기반 속도 계획
            
            # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
            # 평평한 도로인 경우 최대 속도를 계산합니다. 
            # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.
            #print("r : " + str(r))
            #print("friction : " + str(self.road_friction))
            #print("gravityAcc : " + str(gravityAcc))
            v_max = sqrt(r * self.road_friction * gravityAcc)  # 최대 속도 계획
            # print(v_max)
            # print("v_max: " + str(v_max))
            # print("\n")

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(3)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(3)
        # print(out_vel_plan)
        # print("Velocity Planning 길이 *****************************************************")
        # print(len(out_vel_plan))
        # print("***************************************************************************")
        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
