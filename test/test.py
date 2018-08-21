#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def genTwist(lx, ly, lz, ax, ay, az):
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz
    twist.angular.x = ax
    twist.angular.y = ay
    twist.angular.z = az
    return twist


class Test:
    def __init__(self):
        rospy.loginfo('Test node initialized')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def goCircle(self, secs):
        rospy.loginfo("Go circle for {0:d} secs".format(secs))
        # Time minus Duration is a Time
        secs_from_now = rospy.Time.now() + rospy.Duration(secs)

        while not rospy.is_shutdown():
            if rospy.Time.now() < secs_from_now:
                self.pub.publish(genTwist(1, 0, 0, 0, 0, -1))
            else:
                rospy.loginfo("Stop turlebot")
                self.pub.publish(genTwist(0, 0, 0, 0, 0, 0))
                break


def main():
    rospy.init_node('test')
    try:
        test = Test()
        test.goCircle(3)

    except rospy.ROSInterruptException:
        print("ROSInterruptException")


if __name__ == '__main__':
    main()
