#!/usr/bin/env python

# This script creates the /target_distribution for the ergodic controller for the
# specific direct, shared, and auto conditions set as a rosparam in the launch file.
# It subscribes to /input, /visual_dist', and /obj_dist.

##########################
##### Python Imports #####
##########################
import numpy as np
import numpy.random as npr
import matplotlib.pyplot as plt
import copy
import os
import argparse
import sys


##########################
###### ROS Imports #######
##########################
import rospy
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point, Quaternion
import tf

##########################
### File/Local Imports ###
##########################
from user_input.msg import input_array
from user_input.msg import person_position
from dist_autonomy.msg import Target_dist
# from vr_exp_ros.msg import VR_dist

class VR_Swarm_Experiment:

    def __init__(self):
    	rospy.init_node('Experiment_main', anonymous = True)
        self.rate = rospy.Rate(30.0)

        # get control_type rosparameter
        self.control_type = rospy.get_param("/control_type")
        print('/control_type= ',self.control_type)

        # Initialize/set vars
        self.num_drones = 3
        self.grid_lenx = 30
        self.grid_leny = 30

        ## Initialize Publishers/subscribers
        # tanvas subscribers (if needed)
        if ((self.control_type=="direct") or (self.control_type=="shared")):
            self.tan_arr = np.zeros((self.grid_lenx, self.grid_leny))
            self.tan_x, self.tan_y = [], []
            self.tanvas_distr_sub = rospy.Subscriber('/input', input_array, self.update_tan_dist)
        # visual and object subscribers (if needed)
        if ((self.control_type=="auto") or (self.control_type=="shared")):
            self.visual_distr_sub = rospy.Subscriber('/visual_dist', Target_dist, self.update_visual_dist)
            self.obj_distr_sub = rospy.Subscriber('/obj_dist', Target_dist, self.update_obj_dist)

        # Initialize all distributions with zeros
        uniform_dist = 1
        self.tan_arr = np.ones(self.grid_lenx * self.grid_leny)*uniform_dist
        self.obj_dist = np.ones(self.grid_lenx * self.grid_leny)*uniform_dist
        self.visual_dist = np.ones(self.grid_lenx * self.grid_leny)*uniform_dist
        self.obj_dist_times = np.ones(self.grid_lenx * self.grid_leny)
        self.visual_dist_times = np.ones(self.grid_lenx * self.grid_leny)

        # Target Distribution Publisher - initialize with uniform distibution
        self.target_dist = np.ones(self.grid_lenx * self.grid_leny)
        self.target_dist /= np.sum(self.target_dist)
        self.t_dist_pub = rospy.Publisher('/target_distribution', Target_dist, queue_size=1, latch=True)
        self.t_dist_msg = Target_dist()
        # pulblist uniform dist to begin experiment
        self.t_dist_msg.target_array = self.target_dist
        self.t_dist_pub.publish(self.t_dist_msg)
        self.tdist_has_update = False

    def update_obj_dist(self, data):
    # Updates the self.obj_dist array when a new object distribution array is published
        if np.max(data.target_array) > 0.0:
            self.obj_dist = data.target_array
            self.obj_dist /= np.sum(self.obj_dist) # normalize the distribution
            # print('object_dist len', len(data.target_array))
            self.obj_dist_times = self.obj_dist
            self.tdist_has_update = True

    def update_visual_dist(self, data):
    # Updates the self.visual_dist array when a new visual occlusion distribution is published
        if np.max(data.target_array) > 0.0:
            self.visual_dist = data.target_array
            # print('visual_dist len', len(data.target_array))
            self.visual_dist /= np.sum(self.visual_dist) # normalize the distribution
            self.visual_dist_times = self.visual_dist
            self.tdist_has_update = True

    def update_tan_dist(self, data):
    # When a new user input is published, create a distribution from the input and update the self.tan_arr array
        if data.datalen != 0:
            self.tan_x = data.xinput
            self.tan_y = data.yinput
            self.tan_arr = np.ones((self.grid_lenx, self.grid_leny))*.005
            for i in range(data.datalen):
                self.tan_arr[self.tan_x[i], self.tan_y[i]] = 1.0
            self.tan_arr = self.tan_arr.ravel()
            self.tan_arr /= np.sum(self.tan_arr)
            self.tdist_has_update = True

    def update_dist(self):
    # If any of the distributions have been updated, update the target distribution
    # accordingly and depending on the control type
        if self.tdist_has_update == True:
            if (self.control_type=="direct"):
                self.target_dist = self.tan_arr

            elif (self.control_type=="shared"):
                self.target_dist = (3*self.tan_arr + 2*self.visual_dist + 1*self.obj_dist)# * self.visual_dist * self.obj_dist
                if np.max(self.target_dist) > 0.0:
                    self.target_dist /= np.sum(self.target_dist)

            elif (self.control_type=="auto"):
                self.target_dist = (2*self.visual_dist + 1*self.obj_dist) #* self.visual_dist_times * self.obj_dist_times
                if np.max(self.target_dist) > 0.0:
                    self.target_dist /= np.sum(self.target_dist)

            self.t_dist_msg.target_array = self.target_dist
            self.t_dist_pub.publish(self.t_dist_msg)
            self.tdist_has_update = False


if __name__ == '__main__':
    exp = VR_Swarm_Experiment()
    while not rospy.is_shutdown():
        exp.update_dist()
        exp.rate.sleep()
