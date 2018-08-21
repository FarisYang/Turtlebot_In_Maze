#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class BeaconDetect:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        imgHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # define range of colors in HSV (pink, green )
        boundaries = [
            #([125, 100, 30], [255, 255, 255], 'pink hsv'),
            #([33, 80, 40], [102, 255, 255], 'green hsv')
            #([110,50,50], [130,255,255], 'blue hsv'),

            #([86, 31, 4], [220, 88, 50],'blue rgb'),
            #([25, 146, 190], [62, 174, 250],'yellow rgb')

            ([190, 146, 25], [250, 174, 62],'yellow rgb')

        ]

        mask = None

        for (lower, upper, _) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            #bgr
            #pink_boundaries = [(120, 60, 210), (210, 150, 255)]

            #rgb
            # pink_boundaries = [(210, 60, 120), (255, 150, 210)]

            # lower = np.array(pink_boundaries[0], "uint8")
            # upper = np.array(pink_boundaries[1], "uint8")

            # find the colors within the specified boundaries and apply the mask
            #m = cv2.inRange(imgHSV, lower, upper)
            m = cv2.inRange(cv_image, lower, upper)
            mask = m if mask is None else mask | m

        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        cv2.imshow("images", np.hstack([cv_image, output]))
        # cv2.imshow("mask",mask)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = BeaconDetect()
    rospy.init_node('beacon_detect', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
