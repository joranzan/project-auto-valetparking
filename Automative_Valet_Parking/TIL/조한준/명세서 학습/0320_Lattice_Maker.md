# 실패


```python

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList, CtrlCmd
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np

# lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
# 차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
# 충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 경로상의 장애물 탐색
# 3. 좌표 변환 행렬 생성
# 4. 충돌회피 경로 생성
# 5. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
# 6. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
# 7. 선택 된 새로운 지역경로 (/lattice_path) 메세지 Publish

# (0) 필수 학습 지식

# Lattice Planner 는 격자(Lattice) 점을 이용해 충돌 회피 경로를 만드는 예제 입니다.
# Lattice 경로를 만들기 위해 차량 기준 좌표계를 기준으로 경로의 폭과 완만함 정도를 결정합니다.
# Lattice Path 가 만들어질 경로를 3차 방정식을 이용하여 3 차 곡선을 생성 합니다.
# 위 방식을 이용해 차량이 주행 방향으로 회피가 가능한 여러 Lattice 경로 곡선을 만듭니다.
# 만들어진 경로 중 어떤 경로를 주행해야지 안전하기 주행 할 수 있을 지 판단합니다.
# 아래 예제는 경로 상 장애물의 유무를 판단하고 장애물이 있다면 Lattice Path 를 생성하여 회피 할 수 있는 경로를 탐색합니다.


class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        
        # ros Launch File <arg> Tag 
        # ros launch 파일 에는 여러 태그 를 사용 할 수 있지만 
        # 그중 <arg> 태그를 사용하여 변수를 정의 할 수 있습니다.
        # 3 장 에서는 사용하는 Path 정보와 Object 각 예제 별로 다르기 때문에
        # launch 파일의 <arg> 태그를 사용하여 예제에 맞게 변수를 설정합니다.
        
        
        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name,ObjectStatusList, self.object_callback)

        # (1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.

        rospy.Subscriber( "/local_path" , Path, self.path_callback)
        rospy.Subscriber( "/Ego_topic", EgoVehicleStatus, self.status_callback )
        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=10)

        

        self.is_path = False
        self.is_status = False
        self.is_obj = False
        self.num_points_on_curve = 10  # 3차 곡선 경로 상의 점의 수

        rate = rospy.Rate(50) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_obj:
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    #(7) lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        # (2) 경로상의 장애물 탐색
        
        # 경로 상에 존재하는 장애물을 탐색합니다.
        # 경로 상 기준이 되는 지역 경로(local path)에서 일정 거리 이상 가까이 있다면
        # in_crash 변수를 True 값을 할당합니다.

        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:  
                dis = sqrt(pow(obstacle.position.x - path.pose.position.x, 2) + pow(obstacle.position.y - path.pose.position.y, 2))      
                if dis < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                    is_crash = True
                    break

        

        return is_crash

    def collision_check(self, object_data, out_path):
        # (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        
        # 충돌 회피 경로를 생성 한 이후 가장 낮은 비용의 경로를 선택 합니다.
        # lane_weight 에는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
        # 이 중 장애물이 있는 차선에는 가중치 값을 추가합니다.
        # 모든 Path를 탐색 후 가장 비용이 낮은 Path를 선택하게 됩니다.
        # 장애물이 존제하는 차선은 가중치가 추가 되어 높은 비용을 가지게 되기 떄문에 
        # 최종적으로 가장 낮은 비용은 차선을 선택 하게 됩니다. 

        
        
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path 
        
        for obstacle in object_data.obstacle_list:                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))                    
        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg
    
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True


    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        
        if look_distance < 20 : #min 10m   
            look_distance = 20                    

        if len(ref_path.poses) > look_distance :
            # (3) 좌표 변환 행렬 생성
            
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            # (4) Lattice 충돌 회피 경로 생성
            
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
            for end_point in local_lattice_points:
                out_path_per_lane = Path()
                out_path_per_lane.header.frame_id = '/map'

                # 3차 곡선 생성
                a0 = local_ego_vehicle_position[0][0]
                a1 = local_ego_vehicle_position[1][0]
                a2 = (2 * (end_point[1] - a1)) / (look_distance * 2)
                a3 = (3 * (a1 - end_point[1])) / ((look_distance * 2) ** 2)

                for t in np.linspace(0, 1, self.num_points_on_curve):
                    x = a0
                    y = a3 * (t ** 3) + a2 * (t ** 2) + a1 * t
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    # Quaternion을 (0, 0, 0, 1)로 설정하여 방향 정보를 유지합니다.
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 1
                    out_path_per_lane.poses.append(pose)

                    

            # Add_point            
            # 3 차 곡선 경로가 모두 만들어 졌다면 이후 주행 경로를 추가 합니다.
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
            
            # (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

            
        return out_path
        
    def cubic_spline(self, end, start, control, num_points):

        t = np.linspace(0, 1, num_points)
        x = start[0] * (1 - t)**3 + 3 * control[0] * t * (1 - t)**2 + 3 * end[0] * t**2 * (1 - t) + end[0] * t**3
        y = start[1] * (1 - t)**3 + 3 * control[1] * t * (1 - t)**2 + 3 * end[1] * t**2 * (1 - t) + end[1] * t**3
        return list(zip(x, y))

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass


```

```python
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

        
        # ros Launch File <arg> Tag 
        # ros launch 파일 에는 여러 태그 를 사용 할 수 있지만 
        # 그중 <arg> 태그를 사용하여 변수를 정의 할 수 있습니다.
        # 3 장 에서는 사용하는 Path 정보와 Object 각 예제 별로 다르기 때문에
        # launch 파일의 <arg> 태그를 사용하여 예제에 맞게 변수를 설정합니다.
        
        
        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]

        rospy.Subscriber(local_path_name, Path, self.path_callback)

        # (1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber( 'Ego_topic' , EgoVehicleStatus , self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=10)

        

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 20

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            
            rospy.loginfo("Waiting....")
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                prev_time = time.time()

                rospy.loginfo("into While Loop")
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                

                steering = self.calc_pure_pursuit()
                rospy.loginfo("Current Steering Info")
                rospy.loginfo(steering)
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # (8) 제어입력 메세지 Publish
                
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
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
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        current_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_pure_pursuit(self,):

        # (2) 속도 비례 Look Ahead Distance 값 설정
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
        rospy.loginfo(self.lfd)  # 현재 lfd 값 로그로 출력
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #(3) 좌표 변환 행렬 생성
        
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
                angle_to_point = atan2(local_path_point[1], local_path_point[0])
                if abs(angle_to_point) < pi / 2:  # 정면에 있는 경우에만
                    dis = sqrt(local_path_point[0] ** 2 + local_path_point[1] ** 2)
                    if dis >= self.lfd:
                        self.forward_point = local_path_point
                        self.is_look_forward_point = True
                        break


        
        
        # (4) Steering 각도 계산
        
        # 제어 입력을 위한 Steering 각도를 계산 합니다.
        # theta 는 전방주시거리(Look Forward Distance) 와 가장 가까운 Path Point 좌표의 각도를 계산 합니다.
        # Steering 각도는 Pure Pursuit 알고리즘의 각도 계산 수식을 적용하여 조향 각도를 계산합니다.
        theta = atan2(self.forward_point[1], self.forward_point[0])
        steering = atan2( ( 2 * self.vehicle_length * sin(theta)), sqrt(self.forward_point[0]**2 + self.forward_point[1]**2))
        rospy.loginfo("Theta Theta Theta")
        rospy.loginfo(theta)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.15
        self.d_gain = 0.10
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.03

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        # (5) PID 제어 생성
        
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
            print("r : " + str(r))
            print("friction : " + str(self.road_friction))
            print("gravityAcc : " + str(gravityAcc))
            v_max = sqrt(r * self.road_friction * gravityAcc)  # 최대 속도 계획
            print("v_max: " + str(v_max))
            print("\n")            

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan




if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass

```




```python

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList, CtrlCmd
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Path
import numpy as np

# lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
# 차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
# 충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 경로상의 장애물 탐색
# 3. 좌표 변환 행렬 생성
# 4. 충돌회피 경로 생성
# 5. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
# 6. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
# 7. 선택 된 새로운 지역경로 (/lattice_path) 메세지 Publish


# (0) 필수 학습 지식

# Lattice Planner 는 격자(Lattice) 점을 이용해 충돌 회피 경로를 만드는 예제 입니다.
# Lattice 경로를 만들기 위해 차량 기준 좌표계를 기준으로 경로의 폭과 완만함 정도를 결정합니다.
# Lattice Path 가 만들어질 경로를 3차 방정식을 이용하여 3 차 곡선을 생성 합니다.
# 위 방식을 이용해 차량이 주행 방향으로 회피가 가능한 여러 Lattice 경로 곡선을 만듭니다.
# 만들어진 경로 중 어떤 경로를 주행해야지 안전하기 주행 할 수 있을 지 판단합니다.
# 아래 예제는 경로 상 장애물의 유무를 판단하고 장애물이 있다면 Lattice Path 를 생성하여 회피 할 수 있는 경로를 탐색합니다.


class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        
        # ros Launch File <arg> Tag 
        # ros launch 파일 에는 여러 태그 를 사용 할 수 있지만 
        # 그중 <arg> 태그를 사용하여 변수를 정의 할 수 있습니다.
        # 3 장 에서는 사용하는 Path 정보와 Object 각 예제 별로 다르기 때문에
        # launch 파일의 <arg> 태그를 사용하여 예제에 맞게 변수를 설정합니다.
        
        
        # arg = rospy.myargv(argv=sys.argv)
        # object_topic_name = arg[1]

        # rospy.Subscriber(object_topic_name,ObjectStatusList, self.object_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        # (1) subscriber, publisher 선언
        
        # Local/Gloabl Path 와 Odometry Ego Status 데이터를 수신 할 Subscriber 를 만들고 
        # CtrlCmd 를 시뮬레이터로 전송 할 publisher 변수를 만든다.
        # CtrlCmd 은 1장을 참고 한다.
        # Ego topic 데이터는 차량의 현재 속도를 알기 위해 사용한다.
        # Gloabl Path 데이터는 경로의 곡률을 이용한 속도 계획을 위해 사용한다.

        rospy.Subscriber( "/local_path" , Path, self.path_callback)
        rospy.Subscriber( "/Ego_topic", EgoVehicleStatus, self.status_callback )
        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=10)

        

        self.is_path = False
        self.is_status = False
        self.is_obj = False
        self.num_points_on_curve = 10  # 3차 곡선 경로 상의 점의 수

        rate = rospy.Rate(50) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_obj:
                if self.checkObject(self.local_path, self.object_data):
                    lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    #(7) lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, ref_path, object_data):
        # (2) 경로상의 장애물 탐색
        
        # 경로 상에 존재하는 장애물을 탐색합니다.
        # 경로 상 기준이 되는 지역 경로(local path)에서 일정 거리 이상 가까이 있다면
        # in_crash 변수를 True 값을 할당합니다.

        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:  
                dis = sqrt(pow(obstacle.position.x - path.pose.position.x, 2) + pow(obstacle.position.y - path.pose.position.y, 2))      
                if dis < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                    is_crash = True
                    break

        

        return is_crash

    def collision_check(self, object_data, out_path):
        # (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        
        # 충돌 회피 경로를 생성 한 이후 가장 낮은 비용의 경로를 선택 합니다.
        # lane_weight 에는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
        # 이 중 장애물이 있는 차선에는 가중치 값을 추가합니다.
        # 모든 Path를 탐색 후 가장 비용이 낮은 Path를 선택하게 됩니다.
        # 장애물이 존제하는 차선은 가중치가 추가 되어 높은 비용을 가지게 되기 떄문에 
        # 최종적으로 가장 낮은 비용은 차선을 선택 하게 됩니다. 

        
        
        selected_lane = -1        
        lane_weight = [3, 2, 1, 1, 2, 3] #reference path 
        min_weight = float('inf')  # 초기 최소 가중치를 무한대로 설정합니다.
        
        for obstacle in object_data.obstacle_list:                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100


        selected_lane = lane_weight.index(min(lane_weight))           
        # # 최소 가중치를 가진 경로를 선택합니다.
        # for idx, weight in enumerate(lane_weight):
        #     if weight < min_weight:
        #         min_weight = weight
        #         selected_lane = idx           

        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg): ## Vehicle Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg
    
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True


    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        
        if look_distance < 10 : #min 10m   
            look_distance = 10                    

        if len(ref_path.poses) > look_distance :
            # (3) 좌표 변환 행렬 생성
            
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            # (4) Lattice 충돌 회피 경로 생성
            
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
            for end_point in local_lattice_points:
                out_path_per_lane = Path()
                out_path_per_lane.header.frame_id = '/map'

                # 3차 곡선 생성
                a0 = local_ego_vehicle_position[0][0]
                a1 = local_ego_vehicle_position[1][0]
                a2 = (2 * (end_point[1] - a1)) / (look_distance * 2)
                a3 = (3 * (a1 - end_point[1])) / ((look_distance * 2) ** 2)

                for t in np.linspace(0, 1, self.num_points_on_curve):
                    x = a0
                    y = a3 * (t ** 3) + a2 * (t ** 2) + a1 * t
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    # Quaternion을 (0, 0, 0, 1)로 설정하여 방향 정보를 유지합니다.
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 1
                    out_path_per_lane.poses.append(pose)

                out_path.append(out_path_per_lane)
            # Add_point            
            # 3 차 곡선 경로가 모두 만들어 졌다면 이후 주행 경로를 추가 합니다.
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
            
            # (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])

            
        return out_path
        
    def cubic_spline(self, end, start, control, num_points):

        t = np.linspace(0, 1, num_points)
        x = start[0] * (1 - t)**3 + 3 * control[0] * t * (1 - t)**2 + 3 * end[0] * t**2 * (1 - t) + end[0] * t**3
        y = start[1] * (1 - t)**3 + 3 * control[1] * t * (1 - t)**2 + 3 * end[1] * t**2 * (1 - t) + end[1] * t**3
        return list(zip(x, y))

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass




```