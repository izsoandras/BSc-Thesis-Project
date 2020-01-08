#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# Displays the frames received from the camera.
# Only served test purpose

class ImageDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
        self.windowname = "Image window"


    def callback(self,msg):
        #arr = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")
        # Uncomment following two lines for CompressedImage topic
        np_arr = np.fromstring(msg.data, np.uint8)
        cv2.namedWindow(self.windowname)
        rospy.loginfo("DOne")
        cv2.createTrackbar("SLider>",self.windowname,0,100,self.on_trackbar)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow(self.windowname, cv_image)
        cv2.waitKey(1)


    def on_trackbar(self, val):
        rospy.loginfo(val)


if __name__ == "__main__":
    ic = ImageDisplay()
    rospy.init_node("imagedisplay", anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
