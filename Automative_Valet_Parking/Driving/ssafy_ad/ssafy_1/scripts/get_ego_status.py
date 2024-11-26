#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import EgoVehicleStatus

def EgoStatus_callback(data):
    rospy.loginfo('------------------Ego Vehicle Status------------------')
    rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.position.x, data.position.y, data.position.z))
    rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format(data.velocity.x, data.velocity.y, data.velocity.z))
    rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format( data.acceleration.x, data.acceleration.y, data.acceleration.z))
    rospy.loginfo('heading      : {} deg'.format( data.heading))

def listener():
    rospy.init_node('Ego_status_listener', anonymous=True)
    rospy.Subscriber('Ego_topic', EgoVehicleStatus, EgoStatus_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
