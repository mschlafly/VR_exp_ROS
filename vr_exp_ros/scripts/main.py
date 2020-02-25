#!/usr/bin/env python

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
from vr_exp_ros.msg import Target_dist
# from vr_exp_ros.msg import VR_dist

class VR_Swarm_Experiment:

    def __init__(self):
	rospy.init_node('Experiment_main', anonymous = True)

        self.control_type = rospy.get_param("/control_type")
        print('/control_type= ',self.control_type)
        # self.control_type = "ergodic"
        # Initialize/set vars
        self.num_drones = 3
        self.xdim = 30
        self.ydim = 30

        self.grid_lenx = 30
        self.grid_leny = 30

        self.rate = rospy.Rate(30.0)

        # Initialize Publishers/subscribers
        # self.control_type = "shared"
        if ((self.control_type=="direct") or (self.control_type=="shared")):
            self.tan_arr = np.zeros((self.grid_lenx, self.grid_leny))
            self.tan_x, self.tan_y = [], []
            self.tanvas_distr_sub = rospy.Subscriber('/input', input_array, self.update_tan_dist)
        # visual and object subscribers
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
        # self.t_dist_pub = rospy.Publisher('/target_distribution', Target_dist, queue_size=1, latch=True)
        self.t_dist_pub = rospy.Publisher('/target_distribution', Target_dist, queue_size=1, latch=True)

        self.t_dist_msg = Target_dist()
        # t_dist_msg.target_array = self.target_dist
        self.t_dist_msg.target_array = self.target_dist

        self.t_dist_pub.publish(self.t_dist_msg)
        self.tdist_has_update = False




    def update_obj_dist(self, data):
        if np.max(data.target_array) > 0.0:
            self.obj_dist = data.target_array
            self.obj_dist /= np.sum(self.obj_dist)
            print('object_dist len', len(data.target_array))
            self.obj_dist_times = self.obj_dist
            self.tdist_has_update = True

    def update_visual_dist(self, data):
        if np.max(data.target_array) > 0.0:
            self.visual_dist = data.target_array
            print('visual_dist len', len(data.target_array))
            self.visual_dist /= np.sum(self.visual_dist)
            self.visual_dist_times = self.visual_dist
            self.tdist_has_update = True

        #
        # # Create grid over workspace normalized to [0,1] with lenx points
        # grid = np.meshgrid(*[np.linspace(0, 1, self.grid_lenx) for _ in range(2)])
        # gridlist = np.c_[grid[0].ravel(), grid[1].ravel()]
        #
        # # Generate mean(peak) of gaussian using operator's location normalized to [0,1]
        # means = [np.array([data.xpos/self.xdim, data.ypos/self.ydim])]
        # vars  = [np.array([0.1,0.1])**2]
        #
        # # Generate array over workspace for gaussian at operator location
        # val = np.zeros(self.grid.shape[0])
        # for m, v in zip(means, vars):
        #     innerds = np.sum((gridlist-m)**2 / v, 1)
        #     val += np.exp(-innerds/2.0)# / np.sqrt((2*np.pi)**2 * np.prod(v))
        # val += np.random.rand(gridlist.shape[0])*0.1
        # # normalizes the distribution
        # val /= np.sum(val)
        #
        # self.vr_dist = val
        # # self.vr_dist = self.vr_dist.ravel()
        # self.tdist_has_update = True

    def update_tan_dist(self, data):
        if data.datalen != 0:
            self.tan_x = data.xinput
            self.tan_y = data.yinput

            self.tan_arr = np.ones((self.grid_lenx, self.grid_leny))*.005
            for i in range(data.datalen):
                self.tan_arr[self.tan_x[i], self.tan_y[i]] = 1.0
            # self.tan_arr = np.transpose(self.tan_arr)
            self.tan_arr = self.tan_arr.ravel()
            self.tan_arr /= np.sum(self.tan_arr)
            # print("updated tan_arr, shape: ", np.shape(self.tan_arr))
            self.tdist_has_update = True

            # if np.max(self.tan_arr) > 0:
            #     self.tdist_has_update = True

    def update_dist(self):
        if self.tdist_has_update == True:
            # temp = []
            # # if self.vr_dist != []:
            # #     temp = self.vr_dist/np.amax(self.vr_dist) #Normalized to max value of 1
            # if self.control_type is ergodic:
            #     if np.max(self.tan_arr) > 0:
            #         temp = self.tan_arr
            #     # temp = temp + self.tan_arr
            # self.target_dist = temp
            # self.target_dist /= np.sum(self.target_dist)
            # print(self.control_type)
            if (self.control_type=="direct"):
                self.target_dist = self.tan_arr

            elif (self.control_type=="shared"):
                self.target_dist = (3*self.tan_arr + 2*self.visual_dist + 1*self.obj_dist)# * self.visual_dist * self.obj_dist
                if np.max(self.target_dist) > 0.0:
                    self.target_dist /= np.sum(self.target_dist)

            elif (self.control_type=="auto"):
                # if ((np.max(self.visual_dist) > 0) and (np.max(self.obj_dist) > 0)):
                # print("Here \n")
                self.target_dist = (2*self.visual_dist + 1*self.obj_dist) #* self.visual_dist_times * self.obj_dist_times
                if np.max(self.target_dist) > 0.0:
                    self.target_dist /= np.sum(self.target_dist)

            # t_dist_msg = Target_dist()
            # t_dist_msg.target_array = self.target_dist

            # self.t_dist_msg.target_array = np.transpose(self.target_dist)
            self.t_dist_msg.target_array = self.target_dist
            # print('publish /target_distribution')
            self.t_dist_pub.publish(self.t_dist_msg)
            self.tdist_has_update = False


if __name__ == '__main__':
    exp = VR_Swarm_Experiment()
    while not rospy.is_shutdown():
        exp.update_dist()
        exp.rate.sleep()
