#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from grid_map_msgs.msg import GridMap # grid_map_msgs
from vr_exp_ros.msg import Target_dist

class pub_dist:

    '''
    Subscribes to the VR dist topic and constantly plots the distribution for the
    ergodic controller
    '''

    def __init__(self):
        rospy.init_node('visualize', anonymous=True)

        self.target_sub = rospy.Subscriber('/target_distribution', Target_dist, self.update_dist)
        self.grid_pub = rospy.Publisher('/target_dist_grid', GridMap, queue_size=1, latch=True)
        self.rate = rospy.Rate(30)
        self.msg = GridMap()
        self.array = Float32MultiArray()
        x,y = np.meshgrid(np.linspace(0,1,30), np.linspace(0,1,30))
        samples = np.c_[np.ravel(x, order="F"), np.ravel(y,order="F")]
        grid_val = np.exp(-10. * np.sum((samples-np.array([0.2,0.2]))**2, axis=1))*10 \
                + np.exp(-10. * np.sum((samples-np.array([0.8,0.5]))**2, axis=1))*10

        # self.gridarray = grid_val
        self.gridmap = GridMap()
        arr = Float32MultiArray()
        arr.data = grid_val[::-1]
        # print(arr.data.shape)
        # print(np.shape(self.ctrllr._targ_dist.grid_vals[::-1]))
        # arr.data = self.ctrllr._targ_dist.grid_vals[::-1]

        arr.layout.dim.append(MultiArrayDimension())
        arr.layout.dim.append(MultiArrayDimension())

        arr.layout.dim[0].label="column_index"
        arr.layout.dim[0].size=30
        arr.layout.dim[0].stride=30*30

        arr.layout.dim[1].label="row_index"
        arr.layout.dim[1].size=30
        arr.layout.dim[1].stride=30


        self.gridmap.layers.append("elevation")
        self.gridmap.data.append(arr)
        self.gridmap.info.length_x=30
        self.gridmap.info.length_y=30
        self.gridmap.info.pose.position.x=15
        self.gridmap.info.pose.position.y=15

        self.gridmap.info.header.frame_id = "world"
        self.gridmap.info.resolution = 1#0.333333
        self.grid_pub.publish(self.gridmap)
        self.updated_dist = False
        # print("here")

    def update_dist(self, data):
        # print("Recieved published message in pub_grid")
        grids = np.reshape(data.target_array,(30,30))

        # self.gridarray = np.ravel(np.transpose(grids))
        self.gridarray = np.ravel(grids[::-1,::-1].T)
        # self.gridarray = data.target_array
        self.updated_dist = True


    def publish_msg(self):
        # print("here2")
        if self.updated_dist is True:
            # print("publish message in pub_grid")
            self.gridmap.data[0].data = self.gridarray
            self.grid_pub.publish(self.gridmap)
            # self.rate.sleep()
            # print("Published")
            self.updated_dist = False

if __name__ == '__main__':

    x = pub_dist()
    while not rospy.is_shutdown():
        x.publish_msg()
        # rospy.spin()
        rospy.sleep(0.1)
