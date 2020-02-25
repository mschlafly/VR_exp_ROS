#!/usr/bin/env python

##########################
##### Python Imports #####
##########################
import numpy as np
import matplotlib.pyplot as plt

##########################
###### ROS Imports #######
##########################
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

##########################
### File/Local Imports ###
##########################


class Swarm_Listener():

    def __init__(self):

        rospy.init_node('swarm_listener')
        if rospy.has_param('/num_agents'):
            self.num_drones = rospy.get_param('/num_agents')#3
        else:
            self.num_drones = 3

        self.ws_size = 30
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(30.0)
        self.trans = [None]*self.num_drones
        self.rot = [None]*self.num_drones

        # Swarm Publisher
        self.swarm_pub = rospy.Publisher('/swarm_pos', PoseArray, queue_size=1)
        self.swarmpose_msg = PoseArray()


    def listen(self):
        while not rospy.is_shutdown():
            try:
                for i in range(self.num_drones):
                    frame = '/agent'+str(i)
                    (self.trans[i], self.rot[i]) = self.listener.lookupTransform('/world', frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.x = [self.trans[i][0]*self.ws_size for i in range(self.num_drones)]
            self.y = [self.trans[i][1]*self.ws_size for i in range(self.num_drones)]
            self.update_swarm_pos(self.x, self.y)
            self.rate.sleep()

    def update_swarm_pos(self, xpos, ypos):
        
        #### Update PoseArray msg
        self.swarmpose_msg = PoseArray()
        for i in range(self.num_drones):
            pose = Pose()

            pose.position.x = xpos[i]
            pose.position.y = ypos[i]
            pose.position.z = 6.0#(i+1)*2.0

            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            self.swarmpose_msg.poses.append(pose)

        self.swarmpose_msg.header.frame_id = '/world'
        self.swarmpose_msg.header.stamp = rospy.Time.now()
        # Publish Message
        self.swarm_pub.publish(self.swarmpose_msg)
        
if __name__ == '__main__':
    sl = Swarm_Listener()

    try:
        sl.listen()
    except rospy.ROSInterruptException:
        pass
        
