import sys
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class TutleBotPose:

    def __init__(self, name='drax'):
        self.name = name
        rospy.init_node(self.name, anonymous=True)
        
    def listen_odom(self, topic = ''):
        # A subscriber to the topic. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = rospy.Subscriber(topic, Odometry, self._update_odom)
        self.odom = Pose()
        rospy.spin()
    
    def _update_odom(self, data):
        # Callback function which is called when a new message of type Pose is
        # received by the subscriber.
        self.odom = data.pose.pose
        # print into log
        loginfo = "[{}]: pose at ({}, {})".format(self.name, round(self.odom.position.x, 4), round(self.odom.position.y, 4))
        rospy.loginfo(loginfo)


    def listen_pose(self, topic =""):
        self.pose_substriber = rospy.Subscriber(topic, Pose, self._update_pose)
        self.pose = Pose()
        rospy.spin()

    def _update_pose(self, data):
        self.pose = data
        # print into log
        loginfo = "[{}]: pose at ({}, {})".format(self.name, round(self.pose.position.x, 4), round(self.pose.position.y, 4))
        rospy.loginfo(loginfo)

    
class BaconPose:
    def __init__(self):
        pass
    
    def publish_pose(self, topic = 'bacon.pose'):
        pass

    def listen_pose(self, topic = 'bacon.pose'):
        pass

if __name__=='__main__':
    args = sys.argv
    if len(args)==1:
        raise ValueError('Please specialize the method name!')
    else:
        #for i in range(1, len(args)):
        listen_server, topic = args[1], args[2]
        tutleBotPose = TutleBotPose()
        if listen_server == 'listen_odom':
            tutleBotPose.listen_odom(topic)
        if listen_server == 'listen_pose':
            tutleBotPose.listen_pose(topic)

