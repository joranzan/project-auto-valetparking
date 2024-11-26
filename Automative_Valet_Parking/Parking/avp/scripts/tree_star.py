#!/usr/bin/python2
# -*- coding: utf-8 -*-

from queue import Queue

import math

MAX = 987654321

class TreeNode:

    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0.0

    def is_root(self):
        return not self.parent

    # def has_any_children(self):
    #     return True if self.children else False

class Tree:

    def __init__(self, root):
        self.node_list = [root]
        self.size = 0

    def add_node(self, parent, point):
        node = TreeNode(point, parent)
        self.node_list.append(node)
        self.size += 1
        return self.size - 1

    # 출발지점에서 코스트 따지는 거임
    def get_closest_node(self, point, max_range = 6):
        min_distance = MAX
        min_node_idx = self.size - 1

        for idx in range(self.size):
            node = self.node_list[idx]
            node_point = node.point
            local_distance = math.hypot(node_point.x - point.x, node_point.y - point.y)
            # print("dist: {0}".format(local_distance))
            if local_distance > max_range:
                continue
            distance = node.cost + local_distance
            if distance < min_distance:
                min_distance = distance
                min_node_idx = idx
        # index 리턴
        return min_node_idx
    
    def find_near_node_idx(self, new_node_idx, max_range = 1.0):
        near_node = []

        new_node = self.node_list[new_node_idx]
        new_point = new_node.point

        for idx in range(self.size):
            if idx == new_node_idx:
                continue
            
            node = self.node_list[idx]
            
            if idx == node.parent:
                continue
            
            node_point = node.point
            distance = math.hypot(new_point.x - node_point.x, new_point.y - node_point.y)

            if distance <= max_range:
                near_node.append(idx)

        if len(near_node) == 0:
            return None
        
        return near_node
    
    def recalculate(self, parent_idx):
        parent_node = self.node_list[parent_idx]
        parent_node_cost = parent_node.cost
        parent_pnt = parent_node.point

        for idx in range(self.size):
            node = self.node_list[idx]
            pnt = node.point
            if node.parent == parent_idx:
                node.cost = parent_node_cost + math.hypot(parent_pnt.x - pnt.x, parent_pnt.y - pnt.y)
                self.recalculate(idx)

    