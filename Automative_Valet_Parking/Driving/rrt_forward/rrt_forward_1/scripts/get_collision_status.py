#!/usr/bin/env python

import os
import rospy
from morai_msgs.msg import CollisionData

def Collision_callback(data):
    os.system('clear')
    os.system('echo hi')
    for i in range(len(data.collision_object)) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('Collision Object Name : {}'.format(data.collision_object[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.collision_object[i].position.x,data.collision_object[i].position.y,data.collision_object[i].position.z))
def listener():
    rospy.init_node('Collision_listener', anonymous=True)
    rospy.Subscriber('CollisionData', CollisionData, Collision_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
