#!/usr/bin/python2
# -*- coding: utf-8 -*-

from queue import Queue

import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

MAX = 100

class TreeNode:

    def __init__(self, point, parent=None):
        self.children = []
        self.point = point
        self.parent = parent

    def is_root(self):
        return not self.parent

    def has_any_children(self):
        return True if self.children else False

class Tree:

    def __init__(self, root):
        self.root = root
        self.size = 0

    def add_node(self, parent, point):
        node = TreeNode(point, parent)
        parent.children.append(node)
        self.size += 1
        # print(self.size)
        return node

    def get_closest_node(self, point):
        min_distance = MAX
        closest_node = None  # 가장 가까운 노드를 초기화합니다.
        q = Queue()
        q.put(self.root)
        while not q.empty():
            current_node = q.get()
            current_point = current_node.point
            distance = math.sqrt(math.pow(point.x - current_point.x, 2) + math.pow(point.y - current_point.y, 2))
            if distance < min_distance:
                min_distance = distance
                closest_node = current_node
            if current_node.has_any_children():  # 자식 노드가 있는지 확인합니다.
                for child in current_node.children:
                    q.put(child)
        return closest_node