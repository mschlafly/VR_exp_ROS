#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from dist_autonomy.msg import VR_dist

class plot_dist:
    '''
    Subscribes to the VR dist topic and constantly plots the distribution for the
    ergodic controller
    '''

    def __init__(self):
        rospy.init_node('plot_dist', anonymous=True)


        self.getarr = []

        #self.flag = 0
        #print(flag)
        iter = 0;
        if (flag==5):
            for j in range(0,30):
                for i in range(0,30):
                    H[i,j] = iter #self.getarr[iter]
                    iter = iter + 1;
                    print(iter)
                    print(H[i,j])

        plt.imshow(H)
        plt.show()

        self.distr_sub = rospy.Subscriber('/obj_dist', VR_dist, self.update_dist)

            #print(np.array(H))
            #plt.imshow(np.array(H))
            #plt.imshow(H)
            #plt.show()
        #rospy.spin()
        #print(H)

    def update_dist(self, data):

        #self.H = data.arr
        self.getarr = data.arr
        #print("Updating distribution")
        flag = 1
        #print(flag)
        iter = 0;
        for j in range(0,30):
            for i in range(0,30):
                H[i,j] = self.getarr[iter]
                iter = iter + 1;
        print(H)


        #self.fig.canvas.draw_idle()


            # plt.pause(0.01)



if __name__ == '__main__':

    #H = np.array([[1, 2, 3, 4],[5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]])
    #H = np.zeros((4,4))
    #H[1,1] = 100
    #plt.imshow(H)
    #plt.show()
    flag = 0
    H = np.zeros((30,30))
    while not rospy.is_shutdown():
        x = plot_dist()
        # rospy.sleep(1.0)
