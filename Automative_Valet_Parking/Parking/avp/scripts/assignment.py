#!/usr/bin/python2
# -*- coding: utf-8 -*-

import os
from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion,quaternion_from_euler

from tree import Tree, TreeNode

import math
import random
import rospy
import rospkg
from collections import deque

HZ = 5
STEP = 1

BOARD_CORNERS = [-5, 5, 5, -5]

class ROSDemo:
    def __init__(self):
        rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path(pkg_name)
        pkg_path = rospack.get_path('avp')
        full_path = pkg_path + '/path/ssafy_test_path_star.txt'
        # 파일 이름 = ssafy_test_path.txt
        self.f = open(full_path, 'w')
        rospy.init_node('rrt', anonymous=True)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber( '/odom', Odometry, self.odom_callback)
        rospy.Subscriber( 'Ego_topic' , EgoVehicleStatus , self.status_callback)
        rospy.Subscriber("/Object_topic",ObjectStatusList, self.object_callback)

        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.HZ = 5
        self.STEP = 1.5
        self.BOARD_CORNERS = [-5, 5, 5, -5]
        self.frame_count = 0
        # self.obstacles = None
        # self.obstacles_lines = self.get_obstacles_lines(self.obstacles)
        self.car = None
        self.target = Marker()
        self.target_lines = []
        self.point = Marker()
        self.tree_edges = Marker()
        self.p0 = Point()
        self.tree = Tree(TreeNode(0,0))
        # self.collision_edges = self.get_collision_edges_structure()
        self.found_path = False
        self.path_edges = Marker()
        self.drawed_path = False
        self.car_reached = False
        self.path_points = []
        
        # 초기화
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False
        self.is_obj = False
        self.is_done = False

        # First param: topic name; second param: type of the message to be published; third param: size of queued messages,
        # at least 1
        chatter_pub = rospy.Publisher('some_chatter', String, queue_size=10)
        marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        done_pub = rospy.Publisher('done', String, queue_size=10)

        # Each second, ros "spins" and draws 20 frames
        loop_rate = rospy.Rate(30)  # 20hz

        while not rospy.is_shutdown():
            if self.is_odom and self.is_status and self.car is None and self.is_obj:
                self.initialize_objects()

            if self.is_odom == True and self.is_status == True and self.car:
                msg = "Frame index: %s" % self.frame_count
                # rospy.loginfo(msg)
                chatter_pub.publish(msg)

                for obst in self.obstacles:
                    obst.header.stamp = rospy.Time.now()
                    marker_pub.publish(obst)

                self.car.header.stamp = rospy.Time.now()
                self.target.header.stamp = rospy.Time.now()
                self.point.header.stamp = rospy.Time.now()
                self.tree_edges.header.stamp = rospy.Time.now()
                self.collision_edges.header.stamp = rospy.Time.now()
                self.path_edges.header.stamp = rospy.Time.now()

                if self.frame_count % self.HZ == 0 and not self.found_path:
                    rand_pnt = Point()
                    rand_pnt.x = self.x + random.uniform(-30, 30)
                    rand_pnt.y = self.y + random.uniform(-30, 30)
                    rand_pnt.z = 0
                    # 출발지와 목적지 거리를 반지름으로 하고 목적지가 중심인 원 방정식 안에 들어왔을 때
                    # 목적지와 랜덤 포인트의 거리 <= 출발지 목적지 거리
                    dist_start_to_target = math.sqrt(math.pow(self.x - self.target.pose.position.x, 2) + math.pow(self.y - self.target.pose.position.y, 2))
                    dist_random_to_target = math.sqrt(math.pow(rand_pnt.x - self.target.pose.position.x, 2) + math.pow(rand_pnt.y - self.target.pose.position.y, 2))

                    if dist_random_to_target > dist_start_to_target:
                        continue

                    
                    self.point.points = [rand_pnt]

                    close_node = self.tree.get_closest_node(rand_pnt)
                    # print('close_node :', close_node.point)
                    close_pnt = close_node.point
                    # print('closenode : ',close_node.point)
                    total_dist = math.sqrt(math.pow(rand_pnt.x - close_pnt.x, 2) + math.pow(rand_pnt.y - close_pnt.y, 2))
                    dist_ratio = self.STEP / total_dist
                    # print(dist_ratio)
                    new_pnt = Point()
                    new_pnt.x = (1 - dist_ratio) * close_pnt.x + dist_ratio * rand_pnt.x
                    new_pnt.y = (1 - dist_ratio) * close_pnt.y + dist_ratio * rand_pnt.y
                    new_pnt.z = 0

                    if self.collides_object(close_pnt, new_pnt, self.obstacles_lines):
                        self.collision_edges.points.append(close_pnt)
                        self.collision_edges.points.append(new_pnt)
                    else:
                        last_node = self.tree.add_node(close_node, new_pnt)

                        self.tree_edges.points.append(close_pnt)
                        self.tree_edges.points.append(new_pnt)

                    if self.collides_object(close_pnt, new_pnt, self.target_lines):
                        self.found_path = True        

                if self.found_path and not self.drawed_path and not self.is_done:
                    current_node = last_node
                    path_lst = []
                    while not current_node.is_root():
                        self.path_points.append(current_node.point)
                        self.path_edges.points.append(current_node.point)
                        self.path_edges.points.append(current_node.parent.point)
                        current_node = current_node.parent
                        # [('children', [<tree.TreeNode instance at 0x7f31c7c41280>]), ('parent', None), ('point', x: 5.08480935852, y: 1068.8843976, z: 0)]
                        # print(current_node.__dict__.items())
                        data ='{0}\t{1}\t{2}\n'.format(current_node.point.x, current_node.point.y, 0.0)
                        path_lst.append(data)
                        
                        print('Recorded Position:', current_node.point.x, current_node.point.y, current_node.point.z)
                    self.drawed_path = True
                    for path_idx in range(len(path_lst) - 1, -1, -1):
                        data = path_lst[path_idx]
                        print('path : ', data)
                        self.f.write(data)
                    self.f.close()
                    self.is_done = True

                if self.frame_count % 2 == 0 and self.drawed_path and not self.car_reached:
                    # self.path_points.sort()
                    self.car.pose.position = self.path_points.pop()
                    self.car_reached = True if len(self.path_points) == 0 else False

                marker_pub.publish(self.car)
                marker_pub.publish(self.point)
                marker_pub.publish(self.tree_edges)
                marker_pub.publish(self.collision_edges)
                marker_pub.publish(self.path_edges)

                while marker_pub.get_num_connections() < 1:
                    if rospy.is_shutdown():
                        return 0
                    rospy.logwarn_once("Please run Rviz in another terminal.")
                    rospy.sleep(1)

                if self.is_done:
                    print('done?')
                    done_pub.publish('done')
                
                self.frame_count += 1
                loop_rate.sleep()
        self.f.close()
    def initialize_objects(self):
        self.car = self.create_car()
        self.target = self.create_target()
        self.target_lines = self.get_target_lines(self.target)
        self.point = self.get_point_structure()
        self.tree_edges = self.get_tree_edges_structure()
        self.p0 = Point(self.car.pose.position.x, self.car.pose.position.y, 0)
        self.tree = Tree(TreeNode(self.p0))
        self.obstacles = self.create_obstacles()
        self.obstacles_lines = self.get_obstacles_lines(self.obstacles)
        self.collision_edges = self.get_collision_edges_structure()
        self.path_edges = self.get_path_edges_structure()

    def create_obstacles(self):       
        obst_list = []
        
        for obstacle in self.object_data.obstacle_list:
            # print(obstacle)
            obst1 = Marker()
            obst1.type = obst1.CUBE
            obst1.header.frame_id = "map"
            obst1.ns = "obstacles"
            obst1.id = obstacle.unique_id
            obst1.action = obst1.ADD
            obst1.pose.position.x, obst1.pose.position.y, obst1.pose.position.z = obstacle.position.x, obstacle.position.y, self.z
            # obst1.pose.position.x, obst1.pose.position.y, obst1.pose.position.z = 0,0,0
            obst1.pose.orientation.w = 1
            if obstacle.name == "NCAP_GVT":
                obst1.scale.x, obst1.scale.y, obst1.scale.z = 4, 1.8, 1.0
            else:
                obst1.scale.x, obst1.scale.y, obst1.scale.z = 1.0, 1.0, 1.0
            obst1.pose.position.x, obst1.pose.position.y, obst1.pose.position.z = obstacle.position.x, obstacle.position.y, self.z
            # obst1.pose.position.x, obst1.pose.position.y, obst1.pose.position.z = 0,0,0
            obst1.pose.orientation.w = 1
            # obst1.pose.orientation.x = obstacle.heading
            # print('=====================================================')
            # print('o : ', obstacle.heading)
            # 헤딩 값을 orientation으로 변환


            new_heading = obstacle.heading - 27
            if obstacle.heading <= -153:
                new_heading += 360

            # print('heading :',  obstacle.heading)
            # print('tlqkftlqkf :',  new_heading)
            quaternion = quaternion_from_euler(0,0,obstacle.heading)
            quaternion = quaternion_from_euler(0,0,new_heading)
            # print(quaternion)
            _,_,obst1.pose.orientation.z,obst1.pose.orientation.w = quaternion
            
            # print('q' , obst1.pose)
            # print('x :', x)

            obst1.color.r, obst1.color.g, obst1.color.b, obst1.color.a = 0.95, 0.95, 0.95, 1.0
            obst1.lifetime = rospy.Duration()
            obst_list.append(obst1)
            # print(obst1)

        # print(obst_list)
        return obst_list

    def odom_callback(self,msg):
        self.is_odom = True
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        self.z=msg.pose.pose.position.z
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)

    def global_path_callback(self,msg):
        self.is_global_path = True
        self.global_path = msg

    def status_callback(self,msg): 
        self.is_status=True
        self.status_msg=msg  

    # 장애물
    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg
        # print(msg)


    # 차량의 위치
    def create_car(self):
        car = Marker()
        # car = Point()
        car.type = car.CUBE # Marker에서만 활용 가능
        car.header.frame_id = "map"
        car.ns = "car"
        car.id = 0
        car.action = car.ADD
        car.scale.x, car.scale.y, car.scale.z = 0.5, 0.8, 0.5
        car.pose.position.x, car.pose.position.y, car.pose.position.z = self.x, self.y, self.z
        car.pose.orientation.w = 1.0
        car.color.r, car.color.g, car.color.b, car.color.a = 0.45, 0.45, 0.45, 1.0
        car.lifetime = rospy.Duration()
        # TODO 자세 데이터 저장하기

        return car

    # 목표지점 위치
    def create_target(self):
        tgt = Marker()
        tgt.type = tgt.CUBE
        tgt.header.frame_id = "map"
        tgt.ns = "target"
        tgt.id = 0
        tgt.action = tgt.ADD
        tgt.scale.x, tgt.scale.y, tgt.scale.z = 3, 3, 3
        tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = -10.390463829, 1039.50720215, self.z
        # tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = self.x - 10, self.y + 10, self.z
        tgt.pose.orientation.w = 1.0
        tgt.color.r, tgt.color.g, tgt.color.b, tgt.color.a = 0.45, 0.45, 0.45, 1.0
        tgt.lifetime = rospy.Duration()
        return tgt

    def get_tree_edges_structure(self):
        edges = Marker()
        edges.type = edges.LINE_LIST
        edges.header.frame_id = "map"
        edges.ns = "tree_edges"
        edges.id = 0
        edges.action = edges.ADD
        edges.scale.x = 0.02
        edges.pose.orientation.w = 1.0
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = 1.00, 0.95, 0.69, 1.00
        return edges

    def get_point_structure(self):
        point = Marker()
        point.type = point.POINTS
        point.header.frame_id = "map"
        point.ns = "point"
        point.id = 0
        point.action = point.ADD
        point.scale.x, point.scale.y = 0.05, 0.05
        point.pose.orientation.w = 1.0
        point.color.r, point.color.g, point.color.b, point.color.a = 0.45, 0.31, 0.31, 1.00
        return point

    def get_collision_edges_structure(self):
        edges = Marker()
        edges.type = edges.LINE_LIST
        edges.header.frame_id = "map"
        edges.ns = "collision_edges"
        edges.id = 0
        edges.action = edges.ADD
        edges.scale.x = 0.02
        edges.pose.orientation.w = 1.0
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.96, 0.67, 0.67, 1.00
        return edges

    def get_path_edges_structure(self):
        edges = Marker()
        edges.type = edges.LINE_LIST
        edges.header.frame_id = "map"
        edges.ns = "path_edges"
        edges.id = 0
        edges.action = edges.ADD
        edges.scale.x = 0.06
        edges.pose.orientation.w = 1.0
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.35, 0.75, 0.75, 1.00
        return edges

    def get_obstacles_lines(self, obstacles):
        lines = []
        for obst in obstacles:
            scale_x = obst.scale.x
            scale_y = obst.scale.y
            pos_x = obst.pose.position.x
            pos_y = obst.pose.position.y
            if obst.type == obst.CUBE:
                x_i = pos_x - scale_x / 2
                x_f = pos_x + scale_x / 2
                y_i = pos_y - scale_y / 2
                y_f = pos_y + scale_y / 2
                lines.append((Point(x_i, y_i, 0), Point(x_f, y_i, 0)))
                lines.append((Point(x_i, y_f, 0), Point(x_f, y_f, 0)))
                lines.append((Point(x_i, y_i, 0), Point(x_i, y_f, 0)))
                lines.append((Point(x_f, y_i, 0), Point(x_f, y_f, 0)))
        return lines

    def get_target_lines(self, target):
        lines = []
        scale_x = target.scale.x
        scale_y = target.scale.y
        pos_x = target.pose.position.x
        pos_y = target.pose.position.y
        if target.type == target.CUBE:
            x_i = pos_x - scale_x / 2
            x_f = pos_x + scale_x / 2
            y_i = pos_y - scale_y / 2
            y_f = pos_y + scale_y / 2
            lines.append((Point(x_i, y_i, 0), Point(x_f, y_i, 0)))
            lines.append((Point(x_i, y_f, 0), Point(x_f, y_f, 0)))
            lines.append((Point(x_i, y_i, 0), Point(x_i, y_f, 0)))
            lines.append((Point(x_f, y_i, 0), Point(x_f, y_f, 0)))
        return lines

    def orientation(self, p, q, r):
        """
        Check orientation of points p, q, r
        """
        val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
        if val == 0:
            return 0
        return 1 if val > 0 else 2

    def on_segment(self, p, q, r):
        """
        Check if point q lies on segment pr
        """
        if min(p.x, r.x) <= q.x <= max(p.x, r.x) and min(p.y, r.y) <= q.y <= max(p.y, r.y):
            return True
        return False
    
    def calc_dist_to_goal(self, here, goal):
        dx = here.x - goal.pose.position.x
        dy = here.y - goal.pose.position.y
        return math.hypot(dx, dy)

    def collides_line(self, point_i, point_e, line):
        p_i, p_e = line
        o1 = self.orientation(point_i, point_e, p_i)
        o2 = self.orientation(point_i, point_e, p_e)
        o3 = self.orientation(p_i, p_e, point_i)
        o4 = self.orientation(p_i, p_e, point_e)
        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and self.on_segment(point_i, p_i, point_e):
            return True
        if o2 == 0 and self.on_segment(point_i, p_e, point_e):
            return True
        if o3 == 0 and self.on_segment(p_i, point_i, p_e):
            return True
        if o4 == 0 and self.on_segment(p_i, point_e, p_e):
            return True
        return False

    # 장애물 , 차량 방향
    def collides_object(self, point_i, point_e, lines):
        for line in lines:
            if self.collides_line(point_i, point_e, line):
                return True
        return False

if __name__ == '__main__':
    try:
        demo = ROSDemo()
    except rospy.ROSInterruptException:
        pass