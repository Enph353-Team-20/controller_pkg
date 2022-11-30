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

class PlateImage():
    """Class for keeping various versions of the same input image together.
    """
    raw_fr = None
    raw = None
    grey = None    
    cropped = None
    edges = None 
    warped = None

    gp = None # Array of good keypoints

    avg_x = None
    avg_y = None

    base_file_name = 'img' + str(int(time.time()))
    guess = None # NN guess?

    def __init__(self, input):
        self.raw_fr = input.copy()
        self.raw = cv2.resize(self.raw_fr, (int(self.raw_fr.shape[1]*0.5), int(self.raw_fr.shape[0]*0.5)))
        self.grey = cv2.cvtColor(self.raw, cv2.COLOR_BGR2GRAY)



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

        self.max_gp = 0
        self.last_img_save = time.process_time()

        self.good_imgs = []

    def callback(self, data):

        try:
            image_org = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # image_grey = cv2.cvtColor(image_org, cv2.COLOR_BGR2GRAY)
            new_img = PlateImage(image_org)
        except CvBridgeError as e:
            print(e)
        
        # input = cv2.resize(image_grey, (int(image_grey.shape[1]*0.5), int(image_grey.shape[0]*0.5)))
        # self.image_org = cv2.resize(image_org, (int(image_org.shape[1]*0.5), int(image_org.shape[0]*0.5)))

        self.sift.input_image(new_img.grey)
        self.sift.sift_locate()
        
        self.process_img(new_img)

        self.sift.show_debug("matches")

        pass


    def process_img(self, plate_img):
        """Extract plate letters using homography

        Args:
            plate_img (PlateImage): A PlateImage image object
        """
        good_points = self.sift.get_output()

        # If an image has more good points than the current maximum, replace the best previous image
        if (len(good_points) >= self.max_gp):
            self.max_gp = len(good_points)
            avg_x, avg_y = self.sift.centroid_of_points()
            plate_img.avg_x = avg_x
            plate_img.avg_y = avg_y
            self.good_imgs.clear()
            self.good_imgs.append(plate_img)

        if (time.process_time() - self.last_img_save > 2.0) and (len(good_points) == 0 and self.max_gp >= 3):

            for img in self.good_imgs:
                left_bound = max(int(img.avg_x - max(img.avg_x,img.grey.shape[1]-img.avg_x)/img.grey.shape[1]*50), 0)
                right_bound = min(int(img.avg_x + max(img.avg_x,img.grey.shape[1]-img.avg_x)/img.grey.shape[1]*90), img.grey.shape[1])
                top_bound = max(int(img.avg_y - max(img.avg_y,img.grey.shape[0]-img.avg_y)/img.grey.shape[0]*90), 0)
                bottom_bound = min(int(img.avg_y + max(img.avg_y,img.grey.shape[0]-img.avg_y)/img.grey.shape[0]*90), img.grey.shape[0])

                img.cropped = img.raw[top_bound:bottom_bound,left_bound:right_bound,:]

                # print([self.avg_x, self.avg_y, left_bound, right_bound, top_bound, bottom_bound])
                # cv2.circle(img.cropped, (left_bound, top_bound), 5, (0,255,0), -1)
                # cv2.circle(img.cropped, (right_bound, bottom_bound), 5, (0,255,0), -1)
                
                os.chdir('/home/fizzer/Downloads/img_spam')
                cv2.imwrite(img.base_file_name + '_c.png', img.cropped)
                cv2.imshow("cropped", img.cropped)
                cv2.waitKey(1)

                self.perspective_transform_plate(img)
            
            self.max_gp = 0
            self.good_imgs.clear()


    def perspective_transform_plate(self, plate_img):
    
        test_img_hsv = cv2.cvtColor(plate_img.cropped, cv2.COLOR_BGR2HSV)
        denoised = cv2.fastNlMeansDenoising(test_img_hsv, h=5)

        hsv_mask = cv2.inRange(denoised, np.array((0, 0, 100)), np.array((0, 0, 210)))
        hsv_plate = cv2.bitwise_and(test_img_hsv, test_img_hsv, hsv_mask)

        contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        longest_contours = []
        for c in contours:
            if len(c) > 10:
                longest_contours.append(c)

        # print(longest_contours)
            
        edges_img = np.zeros((test_img_hsv.shape[0], test_img_hsv.shape[1], 1),  dtype=np.uint8)

        cv2.drawContours(edges_img, longest_contours, -1, (255,255,255), 1)
        plate_img.edges = edges_img.copy()

        cv2.drawContours(hsv_plate, longest_contours, -1, (255,255,255), 1)
        # cv2.imshow('m', hsv_mask)
        # cv2.imshow('c', hsv_plate)
        # cv2.imshow('e', edges_img)

        linesP = cv2.HoughLinesP(edges_img, rho=1, theta=np.pi / 180, threshold=20, maxLineGap=10, minLineLength=25)

        if len(linesP) == 0:
            print("No Hough lines generated: " + plate_img.base_file_name)
            return
        
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

        if bottommost_line is None or topmost_line is None:
            print("No top or bottom line found: " + plate_img.base_file_name)
            return

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
        warped = cv2.warpPerspective(plate_img.cropped, matrix, (150,450))
        plate_img.warped = warped.copy()
        
        cv2.imshow('w', warped)
        cv2.imshow('h2', hsv_plate)
        cv2.waitKey(1)

        filename = plate_img.base_file_name + '_w.png'
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