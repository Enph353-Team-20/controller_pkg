
#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(data):
    """Analyzes robot camera image to determine location of the line on the ground.
    Args:
        data (Image): Image pulled from camera topic
    """
    
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)


def main(args):
    image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,callback)
    rospy.init_node('camera_test', anonymous=True)
    rate = rospy.Rate(24)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)