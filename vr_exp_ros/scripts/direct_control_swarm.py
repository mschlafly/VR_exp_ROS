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

from user_input.msg import input_array


eps = .5 # distance for status reached
class Swarm_DirectControl():

    def __init__(self):
        rospy.init_node('swarm_direct_controller')
        if rospy.has_param('/num_agents'):
            self.num_drones = rospy.get_param('/num_agents')#3
        else:
            self.num_drones = 3

        self.ws_size = 30
        self.dt = .1
        self.max_vel = 1
        self.max_dist = self.max_vel*self.dt

        self.desired_loc = [None]*self.num_drones
        self.desired_path = [None]*self.num_drones
        self.des_index = [None]*self.num_drones
        rospy.Subscriber('/input', input_array, self.input_location_listener)

        # Swarm Publisher
        self.swarm_loc = np.zeros((self.num_drones, 2))
        self.swarm_pub = rospy.Publisher('/swarm_pos', PoseArray, queue_size=1)
        self.swarmpose_msg = PoseArray()
        self.rate = rospy.Rate(30)

    def input_location_listener(self, data):
        if data.datalen != 0:
            agent_num = data.droneID
            print("Updated desired path for agent {}".format(agent_num))
            self.desired_path[agent_num] = [data.yinput, data.xinput] #flipped for unity coordinte system
            self.desired_loc[agent_num] = [data.yinput[0], data.xinput[0]]
            self.des_index[agent_num] = 0
            # print("Updated des path: ", len(self.desired_path[agent_num][0]),  self.desired_loc[agent_num], [self.desired_path[agent_num][0][0],self.desired_path[agent_num][1][0]])

    def control(self, current_loc, desired_loc):
        dist = desired_loc-current_loc
        if np.linalg.norm(dist) < eps: #if curr_loc == desired_loc
            return current_loc
        else:
            dist = np.array(dist).astype(float)
            dist = dist/np.linalg.norm(dist)
            velocity = dist*self.max_vel
            # if np.any(np.array(dist) > self.max_dist):
            #     dist = dist/np.linalg.norm(dist)
            #     velocity = dist*self.max_vel
            # else:
            #     velocity = dist*self.max_vel
            new_pos = current_loc+velocity*self.dt
            return new_pos

    def generate_traj(self):
        for i in range(self.num_drones):
            if self.desired_loc[i] is not None:
                if np.linalg.norm(self.desired_loc[i]-self.swarm_loc[i]) < eps: #Check if current location is close enough to desired. If so, if desired_path is trajectory, move to next point or restart (if at end of traj). Else, just stay there.
                    if len(self.desired_path[i]) > 1:
                        self.des_index[i] +=1
                        if self.des_index[i] >= len(self.desired_path[i][0]):
                            # print("recycling through")
                            self.des_index[i] = 0

                        self.desired_loc[i] = [self.desired_path[i][0][self.des_index[i]],self.desired_path[i][1][self.des_index[i]]]
                        # print("update desired loc", self.desired_loc[i])
                self.swarm_loc[i] = self.control(self.swarm_loc[i], self.desired_loc[i])

    def update_swarm_pos(self):
        #### Update PoseArray msg
        self.swarmpose_msg = PoseArray()
        self.swarmpose_msg.header.frame_id = "/world"
        self.swarmpose_msg.header.stamp = rospy.Time.now()

        for i in range(self.num_drones):
            pose = Pose()

            pose.position.x = self.swarm_loc[i,0]
            pose.position.y = self.swarm_loc[i,1]
            pose.position.z = 6.0#(i+1)*2.0

            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            self.swarmpose_msg.poses.append(pose)

        # Publish Message
        self.swarm_pub.publish(self.swarmpose_msg)

    def swarm_update(self):
        if np.any(np.array(self.desired_path)!=None):
            self.generate_traj()
        self.update_swarm_pos()


if __name__ == '__main__':
    exp = Swarm_DirectControl()
    while not rospy.is_shutdown():
        exp.swarm_update()
        exp.rate.sleep()
