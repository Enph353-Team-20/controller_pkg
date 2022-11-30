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
        
        self.process_img(input)

        # self.sift.show_debug("homogr")
        self.sift.show_debug("matches")

        pass


    def process_img(self, input):
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

            left_bound = max(int(self.avg_x - max(self.avg_x,self.best_img.shape[1]-self.avg_x)/self.best_img.shape[1]*50), 0)
            right_bound = min(int(self.avg_x + max(self.avg_x,self.best_img.shape[1]-self.avg_x)/self.best_img.shape[1]*90), self.best_img.shape[1])
            top_bound = max(int(self.avg_y - max(self.avg_y,self.best_img.shape[0]-self.avg_y)/self.best_img.shape[0]*90), 0)
            bottom_bound = min(int(self.avg_y + max(self.avg_y,self.best_img.shape[0]-self.avg_y)/self.best_img.shape[0]*90), self.best_img.shape[0])

            cropped_plate = self.best_img_org[top_bound:bottom_bound,left_bound:right_bound,:]

            # print([self.avg_x, self.avg_y, left_bound, right_bound, top_bound, bottom_bound])

            # modified_img = self.best_img
            # cv2.circle(modified_img, (left_bound, top_bound), 5, (0,255,0), -1)
            # cv2.circle(modified_img, (right_bound, bottom_bound), 5, (0,255,0), -1)
            
            filename = 'img' + str(int(time.time())) + '_gp_' + str(self.max_gp) + '.png'
            file = cropped_plate
            os.chdir('/home/fizzer/Downloads/img_spam')
            cv2.imwrite(filename, cropped_plate)
            cv2.imshow("cropped", file)
            cv2.waitKey(1)
            
            self.max_gp = 0

            self.perspective_transform_plate(cropped_plate)


    def perspective_transform_plate(self, input):
    
        test_img_hsv = cv2.cvtColor(input.copy(), cv2.COLOR_BGR2HSV)
        denoised = cv2.fastNlMeansDenoising(test_img_hsv, h=5)

        hsv_mask = cv2.inRange(denoised, np.array((0, 0, 100)), np.array((0, 0, 210)))
        hsv_plate = cv2.bitwise_and(test_img_hsv, test_img_hsv, hsv_mask)

        contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        longest_contours = []
        for c in contours:
            if len(c) > 10:
                longest_contours.append(c)

        print(longest_contours)
            
        edges_img = np.zeros((input.shape[0], input.shape[1], 1),  dtype=np.uint8)

        cv2.drawContours(edges_img, longest_contours, -1, (255,255,255), 1)
        cv2.drawContours(hsv_plate, longest_contours, -1, (255,255,255), 1)
        # cv2.imshow('m', hsv_mask)
        cv2.imshow('c', hsv_plate)
        # cv2.imshow('e', edges_img)

        linesP = cv2.HoughLinesP(edges_img, rho=1, theta=np.pi / 180, threshold=20, maxLineGap=10, minLineLength=25)
        
        topmost_line = None
        bottommost_line = None
        top_y = edges_img.shape[0]
        bottom_y = 0

        for l in linesP:
            l_maxy = max(l[0][1], l[0][3])
            l_miny = min(l[0][1], l[0][3])
            if l_maxy > bottom_y:
                bottom_y = l_maxy
            if l_miny < top_y:
                top_y = l_miny

        for l in linesP:
            if abs(l[0][3] - l[0][1]) <= 5 and abs(top_y - l[0][1]) <= 5: 
                topmost_line = l
            if abs(l[0][3] - l[0][1]) <= 5 and abs(bottom_y - l[0][1]) <= 5: 
                bottommost_line = l

        corners = []
        # Find top left corner first
        if (topmost_line[0][0] < topmost_line[0][2]):
            corners.append((topmost_line[0][0], topmost_line[0][1]))
            corners.append((topmost_line[0][2], topmost_line[0][3]))
        else:
            corners.append((topmost_line[0][2], topmost_line[0][3]))
            corners.append((topmost_line[0][0], topmost_line[0][1]))
        
        # Bottom right corner goes 3rd
        if (bottommost_line[0][0] > bottommost_line[0][2]):
            corners.append((bottommost_line[0][0], bottommost_line[0][1]))
            corners.append((bottommost_line[0][2], bottommost_line[0][3]))
        else:
            corners.append((bottommost_line[0][2], bottommost_line[0][3]))
            corners.append((bottommost_line[0][0], bottommost_line[0][1]))

        for c in corners:
            cv2.circle(hsv_plate, c, 5, (255,255,255), -1)

        dest_pts = np.array([(0, 0), (150,0), (150, 450), (0,450)])
        matrix = cv2.getPerspectiveTransform(np.float32(corners), np.float32(dest_pts))
        warped = cv2.warpPerspective(input, matrix, (150,450))
        
        cv2.imshow('w', warped)
        cv2.imshow('h2', hsv_plate)
        cv2.waitKey(1)

        filename = 'img' + str(int(time.time())) + '.png'
        os.chdir('/home/fizzer/Downloads/img_spam')
        cv2.imwrite(filename, warped)
        cv2.waitKey(1)


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