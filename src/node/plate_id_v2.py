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
        self.sift = SiftProcessor(template, 'P finder')

        debug_flags = {
            "template_kp": False,
            "input_kp": False,
            "homogr": True,
            "matches": True
        }
        self.sift.update_debug_flags(debug_flags)

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

        self.image_org = None
        self.max_gp = 0
        self.best_img = []
        self.prev_best_img = None
        self.best_img_org = None
        self.last_img_save = time.process_time()
        self.avg_x = 0
        self.avg_y = 0

    def callback(self, data):

        try:
            image_org = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_grey = cv2.cvtColor(image_org, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)
        
        input = cv2.resize(image_grey, (int(image_grey.shape[1]*0.5), int(image_grey.shape[0]*0.5)))
        self.image_org = cv2.resize(image_org, (int(image_org.shape[1]*0.5), int(image_org.shape[0]*0.5)))
        
        self.sift.input_image(input)
        self.sift.sift_locate()
        
        self.processing_v2(input)

        # self.sift.show_debug("homogr")
        self.sift.show_debug("matches")

        pass

    def processing_v2(self, input):
        """Extract plate letters using homography

        Args:
            input (_type_): _description_
        """
        good_points = self.sift.get_output()

        if (len(good_points) >= self.max_gp):
            self.max_gp = len(good_points)
            self.prev_best_img = self.best_img
            self.best_img = input
            self.best_img_org = self.image_org
            self.avg_x, self.avg_y = self.sift.centroid_of_points()

        if (time.process_time() - self.last_img_save > 2.0) and (len(good_points) == 0 and self.max_gp >= 3):
            modified_img = self.best_img

            left_bound = max(int(self.avg_x - max(self.avg_x,self.best_img.shape[1]-self.avg_x)/self.best_img.shape[1]*50), 0)
            right_bound = min(int(self.avg_x + max(self.avg_x,self.best_img.shape[1]-self.avg_x)/self.best_img.shape[1]*90), self.best_img.shape[1])
            top_bound = max(int(self.avg_y - max(self.avg_y,self.best_img.shape[0]-self.avg_y)/self.best_img.shape[0]*90), 0)
            bottom_bound = min(int(self.avg_y + max(self.avg_y,self.best_img.shape[0]-self.avg_y)/self.best_img.shape[0]*90), self.best_img.shape[0])

            cropped_plate = self.best_img_org[top_bound:bottom_bound,left_bound:right_bound,:]

            print([self.avg_x, self.avg_y, left_bound, right_bound, top_bound, bottom_bound])

            # cv2.circle(modified_img, (left_bound, top_bound), 5, (0,255,0), -1)
            # cv2.circle(modified_img, (right_bound, bottom_bound), 5, (0,255,0), -1)
            
            filename = 'img' + str(int(time.time())) + '_gp_' + str(self.max_gp) + '.png'
            file = cropped_plate
            os.chdir('/home/fizzer/Downloads/img_spam')
            cv2.imwrite(filename, cropped_plate)
            cv2.imshow("cropped", file)
            cv2.waitKey(1)
            
            self.max_gp = 0


def main(args):
    rospy.init_node('plate_id', anonymous=True)
    rate = rospy.Rate(24)

    template_img = cv2.imread("/home/fizzer/ros_ws/src/controller_pkg/src/node/media/P01.png", cv2.IMREAD_GRAYSCALE)

    # test_img = cv2.imread("/home/fizzer/Downloads/Plate_colourful.png", cv2.IMREAD_UNCHANGED)
    # test_img_hsv = cv2.cvtColor(test_img, cv2.COLOR_BGR2HSV)
    # hsv_mask = cv2.inRange(test_img_hsv, np.array((0, 0, 100)), np.array((5, 5, 105)))
    # print(hsv_mask.shape)
    # print(test_img_hsv.shape)
    # hsv_plate = cv2.bitwise_and(test_img, test_img, hsv_mask)
    # cv2.imshow('m', hsv_mask)
    # cv2.imshow('c', hsv_plate)
    # cv2.waitKey(1)

    # test_img = cv2.imread("/home/fizzer/Downloads/img1669179552_gp_4.png", cv2.IMREAD_GRAYSCALE)
    # ret, thresh = cv2.threshold(test_img, ) 
    # cv2.imshow('c', test_img)
    # cv2.waitKey(1)
 
    processor = PlateID([template_img])


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)