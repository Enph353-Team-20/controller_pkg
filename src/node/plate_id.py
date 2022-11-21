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
import os

from utils.sift_processor import SiftProcessor

class PlateID():

    def __init__(self, template):
        self.sift = SiftProcessor(template)
        debug_flags = {
            "template_kp": True,
            "input_kp": False,
            "homogr": True,
            "matches": True
        }
        self.sift.update_debug_flags(debug_flags)

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

        self.max_gp = 0
        self.best_img = []
        self.prev_best_img = []
        self.img_counter = 0
        self.last_img_save = time.process_time()
        self.avg_x = 0
        self.avg_y = 0

    def callback(self, data):


        try:
            image_org = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        
        input = cv2.resize(image_org, (int(image_org.shape[1]*0.5), int(image_org.shape[0]*0.5)))
        self.sift.input_image(input)
        self.sift.sift_locate()
        good_points = self.sift.get_output()

        if (len(good_points) >= self.max_gp):
            self.max_gp = len(good_points)
            self.prev_best_img = self.best_img
            self.best_img = input
            self.avg_x, self.avg_y = self.sift.centroid_of_points()

        if (time.process_time() - self.last_img_save > 2.0):
            if (len(good_points) == 0 and self.max_gp >= 3): #if we have lost the car
                
                modified_img = self.best_img

                left_bound = max(int(self.avg_x - max(self.avg_x,self.best_img.shape[1]-self.avg_x)/self.best_img.shape[1]*30), 0)
                right_bound = min(int(self.avg_x + max(self.avg_x,self.best_img.shape[1]-self.avg_x)/self.best_img.shape[1]*70), self.best_img.shape[1])
                top_bound = max(int(self.avg_y - max(self.avg_y,self.best_img.shape[0]-self.avg_y)/self.best_img.shape[0]*70), 0)
                bottom_bound = min(int(self.avg_y + max(self.avg_y,self.best_img.shape[0]-self.avg_y)/self.best_img.shape[0]*70), self.best_img.shape[0])

                print([self.avg_x, self.avg_y, left_bound, right_bound, top_bound, bottom_bound])

                # cv2.circle(modified_img, (left_bound, top_bound), 5, (0,255,0), -1)
                # cv2.circle(modified_img, (right_bound, bottom_bound), 5, (0,255,0), -1)
                
                filename = 'img' + str(int(time.time())) + '_gp_' + str(self.max_gp) + '.png'
                os.chdir('/home/fizzer/Downloads')
                cv2.imwrite(filename, modified_img[top_bound:bottom_bound,left_bound:right_bound])
                # cv2.imwrite(filename, modified_img)
                
                # filename = 'img' + str(int(time.time())) + '_gp_' + str(self.max_gp) + 'prev.png'
                # cv2.imwrite(filename, self.prev_best_img)

                self.img_counter += 1
                self.max_gp = 0
                print("Saved image.")


        # self.sift.show_debug("homogr")
        self.sift.show_debug("matches")

        pass



def main(args):
    rospy.init_node('plate_id', anonymous=True)
    rate = rospy.Rate(24)

    template_img = cv2.imread("/home/fizzer/ros_ws/src/controller_pkg/src/node/media/P01.png", cv2.IMREAD_GRAYSCALE)

    processor = PlateID([template_img])


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)