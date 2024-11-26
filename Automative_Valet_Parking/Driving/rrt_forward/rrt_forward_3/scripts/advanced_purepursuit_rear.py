#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,EventInfo, ObjectStatusList
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
        # rospy.Subscriber("/Object_topic",ObjectStatusList, self.object_callback)
        rospy.Subscriber('/ready_to_park', String, self.ready_to_park_callback)
        self.mode_pub = rospy.Publisher('mode_cmd', EventInfo, queue_size=10)
        self.mode_msg = EventInfo()
        self.mode_msg.gear = 2

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
        self.is_gear_change = False
        # 후진할 때
        self.is_rear = False
        self.is_look_rear_point = False

        self.forward_point = Point()       # 전방 노드 객체
        self.rear_point = Point()          # 후방 노드 객체
        self.current_postion = Point()     # 현재 노드 객체


        self.vehicle_length = 2.6     # 차량 길이 
        self.lfd = 6                  # 전방주시거리 [m]
        # self.lfd = 8                  # 전방주시거리 [m]
        self.min_lfd = 1            # 최소 전방주시거리 [m]
        self.max_lfd = 10            # 최대 전방주시거리[m]
        self.lfd_gain = 0.78          # 전방주시거리 증가율(?) -> 적절한 lfd_gain값을 찾아야함
        self.target_velocity = -4     # 최대 속도[km/h]

        self.pid = pidControl() # p_gain, i_gain, d_gain, prev_error, i_control, controlTime


        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown(): # ROS 시스템이 종료되지 않는 한 계속 실행
            if self.is_global_path == True and self.is_path == True and self.is_odom == True and self.is_status == True and self.is_parking_started==True:


                steering = self.calc_pure_pursuit()
                self.mode_msg.gear = 2
                self.ctrl_cmd_msg.steering = steering
                if not self.is_look_rear_point : 
                    rospy.loginfo("no found forward point") #못 찾은 경우
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.brake = 1
                    self.mode_msg.gear = 1

                # PID 컨트롤러를 사용하여 가속도 또는 제동을 계산합니다.
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                # print(output)
                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                if self.is_gear_change:
                    self.ctrl_cmd_msg.brake = 1
                    self.ctrl_cmd_msg.accel = 0.0
                    if self.status_msg.velocity.x < 0.01:
                        self.is_gear_change = False

                # (8) 제어입력 메세지 Publish
                
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                print(self.mode_msg.gear)
                self.mode_pub.publish(self.mode_msg)

                
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

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg
    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def ready_to_park_callback(self, msg):
        if msg.data == "Completed":
            self.is_parking_started = True

    def calc_pure_pursuit(self,):


        self.lfd = self.lfd_gain * self.status_msg.velocity.x  #적절한 lfd값 찾아야함
        self.lfd = max(self.min_lfd, min(self.max_lfd, self.lfd))  # 최소값과 최대값 사이의 값으로 조정
        # rospy.loginfo(self.lfd)  # 현재 lfd 값 로그로 출력

        
        vehicle_position=self.current_postion # 이전에 계산된 현재 차량 위치를 가져옴
        self.is_look_forward_point= False # 전방 주시 지점을 아직 찾지 않았음
        self.is_look_rear_point = False

        translation = [vehicle_position.x, vehicle_position.y]


        # 차량 위치를 기준으로 좌표 변환을 수행하기 위해 변환 행렬을 생성
        trans_matrix = np.array([[cos(self.vehicle_yaw),-sin(self.vehicle_yaw) ,0],
                                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),0],
                                [0,0,1]])

        # 변환 행렬의 역행렬을 계산
        det_trans_matrix = np.linalg.inv(trans_matrix)
        print(len(self.path.poses))
        # 경로 상의 각 점에 대해 반복
        for num,i in enumerate(self.path.poses) :
            # 해당 경로 점의 위치
            path_point = i.pose.position
             # 차량 좌표계로 점을 변환
            global_path_point = [path_point.x - vehicle_position.x, path_point.y - vehicle_position.y, 1] # 3차원 벡터
            local_path_point = det_trans_matrix.dot(global_path_point) #차량 좌표계로 변환한 점

            print("Local Path Point")
            print(local_path_point[0])
            print(local_path_point[1])
            if local_path_point[0] < 0:
                angle_to_point = atan2(local_path_point[1], local_path_point[0])
                print("Angle to Point", angle_to_point)
                if abs(angle_to_point) > pi / 2 and abs(angle_to_point) < pi:  # 후방에 있는 경우에만
                    dis = sqrt(local_path_point[0] ** 2 + local_path_point[1] ** 2)
                    print("dist : ", dis)
                    print("self.lfd : ", self.lfd)
                    if dis >= self.lfd: # 전방주시거리보다 멀리 떨어진 점 찾기
                        self.rear_point = local_path_point #전방주시지점으로 선택된 점 저장
                        self.is_look_rear_point = True #전방주시지점 찾기 완료!
                        self.is_rear = True
                        break
            

        theta = atan2(self.rear_point[1], abs(self.rear_point[0]))
       
        steering = atan2( ( 2 * self.vehicle_length * sin(theta)), sqrt(self.rear_point[0]**2 + self.rear_point[1]**2))

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
        error = abs(error)


        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * ((error - self.prev_error) / self.controlTime)

        output = p_control + self.i_control + d_control
        self.prev_error = error


        return output

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
