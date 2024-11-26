#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import ObjectStatusList

def Object_callback(data):
    os.system('clear')
    rospy.loginfo('-------------------- NPC Vehicle -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_npcs))
    for i in range(data.num_of_npcs) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.npc_list[i].name))
        rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.npc_list[i].position.x,data.npc_list[i].position.y,data.npc_list[i].position.z))
        rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.npc_list[i].velocity.x,data.npc_list[i].velocity.y,data.npc_list[i].velocity.z))
        rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format(data.npc_list[i].acceleration.x,data.npc_list[i].acceleration.y,data.npc_list[i].acceleration.z))
        rospy.loginfo('heading      : {} deg'.format(data.npc_list[i].heading))
        rospy.loginfo('size         : x = {0} , y = {1}, z = {2} m'.format(data.npc_list[i].size.x,data.npc_list[i].size.y,data.npc_list[i].size.z))
    
    rospy.loginfo('-------------------- Pedestrian -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_pedestrian))
    for i in range(data.num_of_pedestrian) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))

    rospy.loginfo('-------------------- Obstacle -------------------------')
    rospy.loginfo('NPC num :{}'.format(data.num_of_obstacle))
    for i in range(data.num_of_obstacle) :
        rospy.loginfo('--------------------Num {}-------------------------'.format(i))
        rospy.loginfo('name : {}'.format(data.obstacle_list[i].name))
        
    

def listener():

    rospy.init_node('Obj_status_listener', anonymous=True)

    rospy.Subscriber( 'Object_topic' , ObjectStatusList , Object_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
