#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from dist_autonomy.msg import object_position

class pub_obj:
    '''
    Subscribes to the VR dist topic and constantly plots the distribution for the
    ergodic controller
    '''

    def __init__(self):
        rospy.init_node('pub_obj', anonymous=True)
        self.obj_pub = rospy.Publisher('/object_position', object_position, queue_size=1)
        self.rate = rospy.Rate(1)
        self.msg = object_position()
        self.msg.id = 1
        self.msg.xpos = 13
        self.msg.ypos = 7
        self.msg.weight = 2

    def publish_msg(self):

        self.msg.id += 1
        self.msg.ypos += 3
        self.obj_pub.publish(self.msg)
        self.rate.sleep()
        print("Published")

if __name__ == '__main__':

    x = pub_obj()
    while not rospy.is_shutdown():
        x.publish_msg()
        rospy.sleep(20.0)
