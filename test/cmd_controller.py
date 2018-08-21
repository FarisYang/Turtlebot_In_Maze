#! /usr/bin/python
"""
A basic ros node written in python that publishes commands typed into
/cmd. Other nodes interested in this command should listen to this topic,
parse the string in the callback and do something. 

Type of the message is a ros std_msgs String

You can expand this to parse and send/publish other things too such as goals,
or twist(drive) commands. 
"""

import rospy
# Note - for debugging you can also do something similar here with goal. 
# Send a PoseStamped into the goal topic. 
# from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Header
import os
import roslib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion

from nav_msgs.msg import Odometry

from ass1.msg import BeaconList

gohome = False
num_beacons = 0
home = None
done_init = None

def callback(data):
    global gohome, home, done_init

    if home is None:
        home = data.pose.pose

    if not gohome:
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        #rospy.loginfo("Setting Pose")
        p   = PoseWithCovarianceStamped()
        msg = PoseWithCovariance()
        #print data.pose.pose
        msg.pose = data.pose.pose #Pose(Point(2.64534568787, 4.98263931274, 0.000), Quaternion(0.000, 0.000, -0.990151822567, 0.139997750521))
        msg.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p.pose = msg
        pub.publish(p)
        

def callback_beacon_list(data):
    global num_beacons

    if len(data.foundBeacons) > num_beacons:
        num_beacons = len(data.foundBeacons)
        rospy.loginfo(data.foundBeacons)
        rospy.loginfo("Number beacons found %d" % (num_beacons))
   

def send_goal_loop():
    global gohome, home

    cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)

    while True:
        # Wait for a user to print something. 
        command = raw_input()

        # Clean-ish exit
        if command == '\n':
            break
        elif command == '':
            break
        elif command == 'nav':
            os.system("rosrun map_server map_saver -f ~/map")
            os.system("roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml")
            break
        elif command == 'home':
            gohome = True
            #os.system("rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: \"map\"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.1}}}'")
            h = Header()
            h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
            p = PoseStamped(h, home)
            pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=100)
            pub.publish(p)
            break
        else: 
            print "Sending command: " + command
            # Publishes the command. 
            cmd_pub.publish(command)

        # publish message here
        rospy.Rate(10).sleep()


if __name__ == "__main__":
    rospy.init_node('command_controller_node')
    rospy.Subscriber('beacon_list', BeaconList, callback_beacon_list)
    rospy.Subscriber('odom', Odometry, callback)
    send_goal_loop()
