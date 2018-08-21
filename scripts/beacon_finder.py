#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from threading import RLock
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from ass1.msg import Beacon, BeaconList, RobotPose
from ass1.srv import StopMonitorCamera, StopMonitorCameraResponse, GetRobotPose, GetRobotPoseResponse


def get_header():
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'map'
    return h


def get_marker(bc, color=None, text=None):
    if color == None:
        color = ColorRGBA(1.0, 0.0, 0.0, 0.8) # red
    else:
        color = ColorRGBA(0.0, 1.0, 0.0, 0.8) # green

    if text == None:
        text = str(bc.id)
        
    marker = Marker(
        id=bc.id,
        type=Marker.TEXT_VIEW_FACING,
        text=text,
        lifetime=rospy.Duration(0),
        pose=Pose(Point(bc.x, bc.y, 1.45), Quaternion(0, 0, 0, 1)),
        scale=Vector3(0.5, 0.5, 0.5),
        header=Header(frame_id='map'),
        color=color)
    return marker


def angle_to_ind(theta, data):
    # converts an angle in radians to a data range index
    #min_angle = -(3 * math.pi) / 4
    min_angle = 0 #-(3 * math.pi) / 4
    angle_dif = theta - min_angle
    ind = int(round(angle_dif / data.angle_increment))
    return ind


def angle_to_dist(angle, data):
    # gets the distance of the closest object at an angle in radians
    ind = angle_to_ind(angle, data)
    print("ind",ind)
    dist = data.ranges[ind]
    #return dist
    return dist-0.2


class BeaconFinder:
    def __init__(self):
        rospy.init_node('beacon_finder', anonymous=True, log_level=rospy.DEBUG)

        self.found = [] # store beacons list found (Beacon[])
        self.found_list_lock = RLock() # Reentrant lock for found[]
        self.stop_monitor = False # for StopMonitorCamera service
        self.robot_pose = None # current robot pose (PoseStamp-map)
        self.laser_scan = None # current laser scan (LaserScan)
        self.bridge = CvBridge() # convert img to cv2
        self.tf_listener = tf.TransformListener() #  transfomation between frame
        self.path = Path(header=get_header(), poses=[]) # for /comp3421/path topic

        rospy.Subscriber("/camera/color/image_raw", Image, self.callback_camera)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.Service('stop_monitor_camera', StopMonitorCamera, self.handle_stop_monitor)
        rospy.Service('get_robot_pose', GetRobotPose, self.handle_get_robot_pose)

        self.beacon_pub = rospy.Publisher('beacon_list', BeaconList, queue_size=10)
        self.marker_pub = rospy.Publisher('/comp3431/beacons', Marker, queue_size=10)
        self.path_pub = rospy.Publisher("/comp3431/path", Path, queue_size=10)
        
        rospy.loginfo('Image recognition started') 

    # publish path when scan
    def callback_scan(self, data):
        self.laser_scan = data

        if not rospy.is_shutdown():
            p, o = self.get_robot_transform()
            # convert to pose
            position = Point(p[0], p[1], p[2])
            orientation = Quaternion(o[0], o[1], o[2], o[3])
            pose = Pose(position, orientation)
            # publish path
            self.path.poses.append(PoseStamped(get_header(), pose))
            try:
                self.path_pub.publish(self.path)
            except KeyboardInterrupt:
                pass

    def handle_stop_monitor(self, request):
        self.stop_monitor = True
        rospy.loginfo('Image recognition stopped monitoring camera')
        return StopMonitorCameraResponse()

    def handle_get_robot_pose(self, request):
        p, o = self.get_robot_transform()
        rp = RobotPose(p[0], p[1], p[2], o[0], o[1], o[2], o[3])
        rospy.loginfo('Returning robot position (%f, %f, %f) orientation (%f, %f, %f, %f)' %(
            rp.px, rp.py, rp.pz, rp.ox, rp.oy, rp.oz, rp.ow))
        return GetRobotPoseResponse(rp)

    def get_robot_transform(self):
        map_frame = '/map'
        robot_frame = '/base_link'
        # getLatestCommonTime(source_frame, target_frame) -> time
        # Determines that most recent time for which Transformer can compute the transform 
        # between the two given frames, that is, between source_frame and target_frame. 
        # Returns a rospy.Time. 
        self.tf_listener.waitForTransform(map_frame, robot_frame, rospy.Time(), rospy.Duration(5))
        t = self.tf_listener.getLatestCommonTime(robot_frame, map_frame) 
        # lookupTransform(target_frame, source_frame, time) -> position, quaternion
        # Returns the transform from source_frame to target_frame at the time time. 
        # time is a rospy.Time instance. 
        # Returned as position (x, y, z) and an orientation quaternion (x, y, z, w). 
        position, orientation = self.tf_listener.lookupTransform(map_frame, robot_frame, t)
        return position, orientation

    def callback_camera(self, data):
        if self.stop_monitor:
            return
        try:
            self.check_for_beacons(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError, e:
            rospy.logerr(e)
        if not rospy.is_shutdown():
            try:
                # Publish list of found beacons
                self.beacon_pub.publish(BeaconList(self.found))
            except KeyboardInterrupt:
                pass

    def is_found(self, beacon):
        for bc in self.found:
            if bc.topColour == beacon.topColour and bc.bottomColour == beacon.bottomColour:
                return True
        return False

    def check_for_beacons(self, im):
        # rows, cols, chs = im.shape
        # if len(self.found) == 0:
        #     print im.shape
        _ , cols, _ = im.shape
        region = im  # im[0:(rows / 2), 0:cols] # Cut image in half, only process top half

        # BGR lower and upper range for pink
        pink_boundaries = [(120, 60, 210), (210, 150, 255)]

        lower = np.array(pink_boundaries[0], "uint8")
        upper = np.array(pink_boundaries[1], "uint8")

        # binary image, white for in range, black otherwise
        mask = cv2.inRange(region, lower, upper)

        # DEBUG
        # cv2.imshow("pink", mask)
        # cv2.waitKey(1)

        # Finding contours: RETR_EXTERNAL for external contours only, CHAIN_APPROX_SIMPLE for approximation
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_threshold = 20

        for cnt in contours:
            # Only look at contours with areas above a certain threshold
            if cv2.contourArea(cnt) > area_threshold:
                # rospy.loginfo(cv2.contourArea(cnt))

                leftmost = tuple(cnt[cnt[:, :, 0].argmin()][0])
                rightmost = tuple(cnt[cnt[:, :, 0].argmax()][0])
                topmost = tuple(cnt[cnt[:, :, 1].argmin()][0])
                bottommost = tuple(cnt[cnt[:, :, 1].argmax()][0])

                left_x = leftmost[0]
                right_x = rightmost[0]
                top_y = topmost[1]
                bottom_y = bottommost[1]

                # Check region above and below pink region
                top_region = region[:top_y, left_x:right_x]
                bottom_region = region[bottom_y:, left_x:right_x]

                bc = self.check_region(top_region, 0)
                if bc.topColour == 'none':
                    bc = self.check_region(bottom_region, 1)
                if bc.topColour == 'none':
                    #rospy.loginfo("False positive for pink")
                    continue
                
                with self.found_list_lock:  # read-write lock for found list
                    if not self.is_found(bc):
                        # Calculate angle to centre of beacon
                        D = cols / 2 # D = 320 pixel

                        # camera has a 58 degree field-of-view
                        alpha = math.atan(
                            (D - (left_x + right_x) / 2) * 1.0 / D * math.tan(math.radians(30.0)))

                        # beacon_range (d) = beacon distance from TurtleBot = laser.ranges [ beacon index ]
                        while self.laser_scan is None: 
                            rospy.sleep(1) # make sure laser_scan is not None
                        beacon_range = angle_to_dist(alpha, self.laser_scan)
                       
                        # get robot position
                        position, orientation = self.get_robot_transform()

                        # convert from quaternion to angle
                        robot_angle = euler_from_quaternion(orientation)[2]
                        target_angle = robot_angle + alpha # theta2 = theta1 + theta3
    
                        #print("alpha degree:", math.degrees(alpha))
                        #print("distance:", beacon_range)

                        # beacon.x = bot.x + d * cos(theta2)
                        bc.x = position[0] + math.cos(target_angle) * beacon_range  
                        bc.y = position[1] + math.sin(target_angle) * beacon_range

                        bc.id = len(self.found)+1

                        self.found.append(bc)
                        self.marker_pub.publish(get_marker(bc))

                        rospy.loginfo("Beacon %d found at (%f, %f), angle %f, range %f" % (
                            bc.id, bc.x, bc.y, math.degrees(alpha), beacon_range))
                        rospy.loginfo("Robot location at (%f, %f)" % (position[0], position[1]))

                        # DEBUG: show robot pose
                        # bot = bc
                        # bot.id = bc.id*10
                        # bot.x = position[0]
                        # bot.y = position[1]
                        # self.marker_pub.publish(get_marker(bot, color='green', text=str(bc.id))) 
                    else:
                        pass

    @staticmethod
    # Check region for colour, create beacon object
    def check_region(image, region):
        area_threshold = 20

        # BGR lower and upper range for green, blue and yellow
        green_boundaries = [(60, 80, 0), (90, 110, 10)]
        blue_boundaries = [(150, 100, 0), (250, 191, 61)]
        yellow_boundaries = [(0, 140, 160), (70, 240, 255)]

        green_lower = np.array(green_boundaries[0], "uint8")
        green_upper = np.array(green_boundaries[1], "uint8")

        blue_lower = np.array(blue_boundaries[0], "uint8")
        blue_upper = np.array(blue_boundaries[1], "uint8")

        yellow_lower = np.array(yellow_boundaries[0], "uint8")
        yellow_upper = np.array(yellow_boundaries[1], "uint8")

        mask_green = cv2.inRange(image, green_lower, green_upper)
        mask_blue = cv2.inRange(image, blue_lower, blue_upper)
        mask_yellow = cv2.inRange(image, yellow_lower, yellow_upper)

        # cv2.imshow("green", mask_green)
        # cv2.imshow("blue", mask_blue)
        # cv2.imshow("yellow", mask_yellow)
        # cv2.waitKey(1)

        _, contours_green, _ = cv2.findContours(
            mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_green = []

        for cntGreen in contours_green:
            areas_green.append(cv2.contourArea(cntGreen))

        _, contours_blue, _ = cv2.findContours(
            mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_blue = []

        for cntBlue in contours_blue:
            areas_blue.append(cv2.contourArea(cntBlue))

        _, contours_yellow, _ = cv2.findContours(
            mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas_yellow = []

        for cntYellow in contours_yellow:
            areas_yellow.append(cv2.contourArea(cntYellow))

        bc = Beacon()
        # Check maximum area for contours of each colour
        if len(areas_green) != 0 and max(areas_green) > area_threshold:
            if region == 0:
                bc.topColour = 'green'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'green'
        elif len(areas_blue) != 0 and max(areas_blue) > area_threshold:
            if region == 0:
                bc.topColour = 'blue'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'blue'
        elif len(areas_yellow) != 0 and max(areas_yellow) > area_threshold:
            if region == 0:
                bc.topColour = 'yellow'
                bc.bottomColour = 'pink'
            else:
                bc.topColour = 'pink'
                bc.bottomColour = 'yellow'
        else:
            bc.topColour = 'none'
            bc.bottomColour = 'none'

        return bc


def main():
    BeaconFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
