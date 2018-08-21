#! /usr/bin/python

import rospy
from std_msgs.msg import String, Header
import os
import roslib
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from ass1.msg import BeaconList
from ass1.srv import StopMonitorCamera, GetRobotPose


TOTAL_BEACONS = 6


def get_header():
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'map'
    return h


class Run:
    def __init__(self):
        rospy.init_node('ass1_main')

        self.num_beacons = 0
        self.total_beacons = TOTAL_BEACONS
        
        self.cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.map_pub = rospy.Publisher('/comp3431/map', OccupancyGrid, queue_size=10)

        rospy.Subscriber('beacon_list', BeaconList, self.callback_beacon_list)
        rospy.Subscriber('map', OccupancyGrid, self.callback_map)

        self.home_pose = self.get_pose()
        rospy.loginfo(self.home_pose)
        rospy.loginfo("Saved home pose")
        
    def callback_map(self, data):
        if not rospy.is_shutdown():
            try:
                self.map_pub.publish(data)
            except KeyboardInterrupt:
                pass

    def callback_beacon_list(self, data):
        if len(data.foundBeacons) > self.num_beacons:
            self.num_beacons = len(data.foundBeacons)
            rospy.loginfo(data.foundBeacons)
            rospy.loginfo("Number beacons found %d" % (self.num_beacons))

        if self.num_beacons == self.total_beacons:
            self.total_beacons = -1
            self.stop_monitor_camera()
            self.wall_follow('stop')
            self.gohome()

    def wall_follow(self, command):
        rospy.loginfo("Sending command to wall follow: " + command)
        self.cmd_pub.publish(command)

    def get_pose(self):
        rospy.wait_for_service('get_robot_pose')
        try:
            srv = rospy.ServiceProxy('get_robot_pose', GetRobotPose)
            response = srv()
            # convert RobotPose to Pose
            rp = response.pose 
            position = Point(rp.px, rp.py, rp.pz)
            orientation = Quaternion(rp.ox, rp.oy, rp.oz, rp.ow)
            return Pose(position, orientation)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def stop_monitor_camera(self):
        rospy.wait_for_service('stop_monitor_camera')
        try:
            srv = rospy.ServiceProxy('stop_monitor_camera', StopMonitorCamera)
            srv()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def gohome(self):
        rospy.loginfo("Going home")
        self.goal_pub.publish(PoseStamped(get_header(), self.home_pose))


def send_cmd_loop(app):
    while True:
        command = raw_input()  # Wait for a user to print something.

        if command == '\n':
            break
        elif command == '':
            break
        elif command == 'home':
            app.gohome()
        else:
            app.wall_follow(command)

        rospy.Rate(10).sleep()


if __name__ == '__main__':
    app = Run()
    send_cmd_loop(app)
