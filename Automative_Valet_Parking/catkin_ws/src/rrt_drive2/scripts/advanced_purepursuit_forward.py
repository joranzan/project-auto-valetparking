#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,EventInfo
from std_msgs.msg import Int8
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_msgs.msg import Int8, String

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

# (0) 필수 학습 지식

# advanced_purepursuit 은 Pure Pursuit 알고리즘을 강화 한 예제입니다.
# 이전까지 사용한 Pure Pursuit 알고리즘은 고정된 전방주시거리(Look Forward Distance) 값을 사용하였습니다.
# 해당 예제에서는 전방주시거리(Look Forward Distance) 값을 주행 속도에 비례한 값으로 설정합니다.
# 이때 최소 최대 전방주시거리(Look Forward Distance) 를 설정합니다.
# 주행 속도에 비례한 값으로 변경 한 뒤 "self.lfd_gain" 을 변경 하여서 직접 제어기 성능을 튜닝 해보세요.
# 


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #T(1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.

        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("local_path", Path, self.path_callback )
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber( 'Ego_topic' , EgoVehicleStatus , self.status_callback)
        rospy.Subscriber('/ready_to_park', String, self.ready_to_park_callback)
        self.daewy = rospy.Publisher('end_topic', Int8, queue_size=1)
        self.mode_pub = rospy.Publisher('mode_cmd', EventInfo, queue_size=10)
        self.mode_msg = EventInfo()
        self.mode_msg.gear = 4

        # Publish
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=10)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1
        self.is_parking_started = False
        # Trigger
        self.is_path = False      # Local Path를 찾았는지 여부
        self.is_odom = False      # Odometry(Ego차량의 위치, 방향) 받았는지 여부 
        self.is_status = False    # 차량 정보 받았는지 여부
        self.is_global_path = False # Global Path를 찾았는지 여부


        self.is_look_forward_point = False

        self.forward_point = Point()       # 전방 노드 객체
        self.current_postion = Point()     # 현재 노드 객체
        self.status_msg = EgoVehicleStatus()

        self.vehicle_length = 2.6     # 차량 길이 
        self.lfd = 6                  # 전방주시거리 [m]
        # self.lfd = 8                  # 전방주시거리 [m]
        self.min_lfd = 2              # 최소 전방주시거리 [m]
        self.max_lfd = 12            # 최대 전방주시거리[m]
        self.lfd_gain = 0.78          # 전방주시거리 증가율(?) -> 적절한 lfd_gain값을 찾아야함
        self.target_velocity = 4    # 최대 속도[km/h]

        self.pid = pidControl() # p_gain, i_gain, d_gain, prev_error, i_control, controlTime

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown(): # ROS 시스템이 종료되지 않는 한 계속 실행

            if self.is_path == True and self.is_odom == True and self.is_status == True and self.is_parking_started==True:
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path) #차량의 상태, 전역경로

                steering = self.calc_pure_pursuit()
                #전방주시지점 찾은 여부 판별 
                # print(self.is_look_forward_point)
                if self.is_look_forward_point : #찾은 경우에는 steering값 update                        
                    self.ctrl_cmd_msg.steering = steering
                    
                else : 
                    rospy.loginfo("no found forward point") #못 찾은 경우
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.brake = 1
                    self.mode_msg.gear = 1
                    self.mode_pub.publish(self.mode_msg)
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    self.daewy.publish(2)
                    break

                # PID 컨트롤러를 사용하여 가속도 또는 제동을 계산합니다.
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                # print(output)
                if output > 0.0:
                    # print("INTO OUTPUT **********************************")
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    # print("INTO OUTPUT **********************************")
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # (8) 제어입력 메세지 Publish
                
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                # print(self.mode_msg.gear)
                # print(self.status_msg.velocity.x)
                self.mode_pub.publish(self.mode_msg)
                
                # print(self.ctrl_cmd_msg)
                print(self.ctrl_cmd_msg.brake)
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

    def status_callback(self,msg): ## Vehicle Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
    
    def ready_to_park_callback(self, msg):
        if msg.data == "Completed":
            self.is_parking_started = True

    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')  # 현재까지 찾은 최소 거리 
        currnet_waypoint = -1    # 최소 거리에 해당하는 waypoint의 인덱스 나타냄
        for i,pose in enumerate(global_path.poses):   # global_path.poses : 전역 경로의 모든 waypoint
            dx = self.current_postion.x - pose.pose.position.x  
            dy = self.current_postion.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))  # 해당 point까지의 거리
            if min_dist > dist : 
                min_dist = dist 
                currnet_waypoint = i
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
        self.lfd = self.lfd_gain * self.status_msg.velocity.x  #적절한 lfd값 찾아야함
        self.lfd = max(self.min_lfd, min(self.max_lfd, self.lfd))  # 최소값과 최대값 사이의 값으로 조정
        # rospy.loginfo(self.lfd)  # 현재 lfd 값 로그로 출력

        
        vehicle_position=self.current_postion # 이전에 계산된 현재 차량 위치를 가져옴
        self.is_look_forward_point= False # 전방 주시 지점을 아직 찾지 않았음

        translation = [vehicle_position.x, vehicle_position.y]

        # (3) 좌표 변환 행렬 생성
        
        # Pure Pursuit 알고리즘을 실행 하기 위해서 차량 기준의 좌표계가 필요합니다.
        # Path 데이터를 현재 차량 기준 좌표계로 좌표 변환이 필요합니다.
        # 좌표 변환을 위한 좌표 변환 행렬을 작성합니다.
        # Path 데이터를 차량 기준 좌표 계로 변환 후 Pure Pursuit 알고리즘 중 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 찾습니다.
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 이용하여 조향 각도를 계산하게 됩니다.
        # 좌표 변환 행렬을 이용해 Path 데이터를 차량 기준 좌표 계로 바꾸는 반복 문을 작성 한 뒤
        # 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 를 계산하는 로직을 작성 하세요.

        # 차량 위치를 기준으로 좌표 변환을 수행하기 위해 변환 행렬을 생성
        trans_matrix = np.array([[cos(self.vehicle_yaw),-sin(self.vehicle_yaw) ,0],
                                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),0],
                                [0,0,1]])
        # 변환 행렬의 역행렬을 계산
        det_trans_matrix = np.linalg.inv(trans_matrix)

        # 경로 상의 각 점에 대해 반복
        
        for num,i in enumerate(self.path.poses) :
            # 해당 경로 점의 위치
            path_point = i.pose.position
             # 차량 좌표계로 점을 변환
            global_path_point = [path_point.x - vehicle_position.x, path_point.y - vehicle_position.y, 1] # 3차원 벡터
            local_path_point = det_trans_matrix.dot(global_path_point) #차량 좌표계로 변환한 점
            #print("g :", global_path_point)
            #print("l :", local_path_point)
            # if local_path_point[0]>0 :
            #     dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
            #     if dis >= self.lfd :
            #         self.forward_point = local_path_point
            #         self.is_look_forward_point = True
            #         break

            # 변환된 점이 차량 앞에 있는지 확인
            if local_path_point[0] > 0:
                # 각도를 계산하여 정면에 있는지 확인
                angle_to_point = atan2(local_path_point[1], local_path_point[0])
                # print('anglepoint :', angle_to_point)
                if abs(angle_to_point) < pi / 2:  # 정면에 있는 경우에만
                    #전방주시지점 찾기
                    #차량좌표계 기준 차량과 경로점 사이의 거리
                    dis = sqrt(local_path_point[0] ** 2 + local_path_point[1] ** 2)
                    if dis >= self.lfd and abs(angle_to_point) <= pi / 3: # 전방주시거리보다 멀리 떨어진 점 찾기
                        self.forward_point = local_path_point #전방주시지점으로 선택된 점 저장
                        self.is_look_forward_point = True #전방주시지점 찾기 완료!
                        # print(self.forward_point)
                        break
                        #continue

        ### 전방 주시 지점을 찾음
        # self.forward_point
        # print(self.forward_point)
        # (4) Steering 각도 계산
        
        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        theta = atan2(self.forward_point[1], self.forward_point[0])
        #Pure Pursuit 알고리즘에서 조향각 유도식 참고
        steering = atan2( ( 2 * self.vehicle_length * sin(theta)), sqrt(self.forward_point[0]**2 + self.forward_point[1]**2))
        # if self.forward_point[0] < 0:
        #     steering = -steering
        # steering = steering
        # print('ss', steering)
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

        print(output)
        return output


if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass

