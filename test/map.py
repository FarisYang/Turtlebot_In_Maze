#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String


from nav_msgs.msg import OccupancyGrid

import numpy as np
import matplotlib as ml
import matplotlib.pyplot as plt


class MapPlot:
    def __init__(self):
        self.map_sub = rospy.Subscriber(
            "/map", OccupancyGrid, self.callback)
        #self.map = np.array([])

    def callback(self, data):
        w = data.info.width
        h = data.info.height

        arr = np.reshape(np.array(data.data),(w,h))

        left = w
        right = 0
        for i in range(w):
            for j in range(h):
                if arr[i,j] !=-1:
                    if right < i:
                        right = i   
                    if left > i:
                        left = i
        bot = 0
        top = h
        for j in range(h):
            for i in range(w):
                if arr[i,j] !=-1:
                    if bot < j:
                        bot = j   
                    if top > j:
                        top = j
        
        crop_arr = np.zeros((right-left+1,bot-top+1))
        for i in range(left,right+1):
            for j in range(top,bot+1):
                crop_arr[i-left,j-top] = arr[i,j]

        #if self.map is None or not np.array_equal(self.map,crop_arr):
        #    self.map = crop_arr
        plt.figure(1)
        plt.clf()
        plt.imshow(crop_arr)
        plt.show()

def main(args):
    mm = MapPlot()
    rospy.init_node('map_plot', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        plt.close('all')
        print("Shutting down")
    
    
    



if __name__ == '__main__':
    main(sys.argv)
