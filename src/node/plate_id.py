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
import time

from utils.sift_processor import SiftProcessor

class PlateID():

    def __init__(self, template):
        self.sift = SiftProcessor(template)
        debug_flags = {
            "template_kp": True,
            "input_kp": False,
            "homogr": True,
            "matches": False
        }
        self.sift.update_debug_flags(debug_flags)

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)



    def callback(self, data):


        try:
            input = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        
        self.sift.input_image(input)
        self.sift.sift_locate()
        good_points = self.sift.get_output()
        self.sift.show_debug("homogr")

        pass



def main(args):
    rospy.init_node('plate_id', anonymous=True)
    rate = rospy.Rate(24)

    template_img = cv2.imread("/home/fizzer/ros_ws/src/controller_pkg/src/node/media/P02.png", cv2.IMREAD_GRAYSCALE)

    processor = PlateID([template_img])


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)