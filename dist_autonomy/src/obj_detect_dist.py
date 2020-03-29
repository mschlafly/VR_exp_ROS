#!/usr/bin/env python
import rospy
import numpy as np
from dist_autonomy.msg import Target_dist
from dist_autonomy.msg import object_position

# This class updates the a distribution with gaussian peaks around each detected object
class obj_dist:

    def __init__(self):
        rospy.init_node('obj_dist', anonymous=True)

        # Distribution parameters governing how the array is updated
        self.time_limit = 15 # limit in seconds for keeping object in the distribution
        self.var = 2**2 # square this value so it won't need to be squared later
        self.max_radius = 5**2 # the max radius of each gaussian peak (squared)

        # Define list of objects (each with size (5,1)) where the list has length num_obj
        self.num_obj = 0
        self.obj_list = np.zeros((5,1))

        #to keep track the time since the object has been detected
        self.prev_time = rospy.get_time()

        # The size of the grid
        self.grid_size = 30

        rospy.Subscriber('/object_position', object_position, self.update_list)
        self.obj_pub = rospy.Publisher('/obj_dist', Target_dist, queue_size=1)
        self.rate = rospy.Rate(1) # the frequency that distributions are published
        self.msg = Target_dist()

    # Update the /obj_dist depending on the list of detected objects
    # Publish the ravelled array
    def update_dist(self):
        dist = np.zeros((self.grid_size,self.grid_size))
        dt = rospy.get_time() - self.prev_time
        self.prev_time = rospy.get_time()
        obj = 0;
        while (obj<self.num_obj):
            self.obj_list[4,obj] -= dt
            if (self.obj_list[4,obj] < 0):
                self.obj_list = np.delete(self.obj_list, obj, 1) # the 1 means we want to delete a column
                self.num_obj -= 1
            else:
                if (self.num_obj>0):
                    objdist = self.create_peak(self.obj_list[1,obj],self.obj_list[2,obj])
                    dist += self.obj_list[3,obj] * objdist
            obj += 1
        self.msg.target_array = np.ravel(dist)
        self.obj_pub.publish(self.msg)

    # create a Gaussian peak on a 2D grid of (self.grid_size,self.grid_size)
    # at location (xpos, ypos)
    def create_peak(self, xpos, ypos):
        dist = np.zeros((self.grid_size,self.grid_size))
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                dist_squared = ((i+0.5-xpos)**2+(j+0.5-ypos)**2)
                if (dist_squared > self.max_radius):
                    dist_squared = self.max_radius
                dist[i,j] = np.exp(-dist_squared/self.var)
        dist /= np.sum(dist)
        return dist

    # Update the list detected objects. If the object had already been
    # found, update the position.
    def update_list(self, data):
        if (data.xpos>0 or data.ypos>0):
            newID = data.id
            found = False
            for obj in range(self.num_obj):
                if (newID == self.obj_list[0,obj]):
                    found = True
                    self.obj_list[1,obj] = data.xpos
                    self.obj_list[2,obj] = data.ypos
                    self.obj_list[4,obj] = self.time_limit
            if (found == False):
                self.obj_list[0,self.num_obj] = newID
                self.obj_list[1,self.num_obj] = data.xpos
                self.obj_list[2,self.num_obj] = data.ypos
                self.obj_list[3,self.num_obj] = data.weight
                self.obj_list[4,self.num_obj] = self.time_limit
                self.obj_list = np.append(self.obj_list,np.zeros((5,1)),axis=1)
                self.num_obj += 1
        self.update_dist()

if __name__ == '__main__':
    x = obj_dist()
    rospy.spin()
