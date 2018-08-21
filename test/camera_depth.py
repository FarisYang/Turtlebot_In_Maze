import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#never test, but right logic
class BeaconDepth:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.bridge=CvBridge()
        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        print(cv_image[10,10])