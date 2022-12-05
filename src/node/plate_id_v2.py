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

    base_file_name = None
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
            new_img.base_file_name = 'img' + str(int(time.time_ns()))
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
        if (len(good_points) >= self.max_gp and len(good_points) >= 2):
            avg_x, avg_y = self.sift.centroid_of_points()
            plate_img.avg_x = avg_x
            plate_img.avg_y = avg_y
            # self.good_imgs.clear()
            print("Appended")
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
                
                # os.chdir('/home/fizzer/Downloads/img_spam')
                # cv2.imwrite(img.base_file_name + '_c.png', img.cropped)
                # cv2.imshow("cropped", img.cropped)
                # cv2.waitKey(1)

                self.perspective_transform_plate(img)
            
            self.max_gp = 0
            self.good_imgs.clear()


    def findCorners(self, img):

        # convert to hsv format
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # filter the hsv (mask1 is white and mask2 is the license plate gray)
        lower1 = np.array([0,0,90])
        upper1 = np.array([0,10,210])
        mask1 = cv2.inRange(test_img_hsv,lower1,upper1)
        # lower2 = np.array([90,1,70])
        # upper2 = np.array([120,60,180])
        # mask2 = cv2.inRange(test_img_hsv,lower2,upper2)
        # hsv_mask = cv2.bitwise_or(mask1,mask2)
        hsv_mask = mask1

        # get the contours
        contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # find the largest contour
        largest_contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]

        # plot the contour on screen
        edges = np.zeros((hsv.shape[0], hsv.shape[1], 1),  dtype=np.uint8)
        cv2.drawContours(edges, largest_contour, -1, (255, 255, 255), 1)
        cv2.imshow('edges', edges)

        # get the corners of the largest contour
        epsilon = 0.05*cv2.arcLength(largest_contour,True)
        box = cv2.approxPolyDP(largest_contour, epsilon, True)

        # see if there are 4 corners
        if box.shape[0] != 4:
            return (None, 0)
        
        # if there are 4 corners then sort them
        box2 = np.zeros((4,2))
        for i in range(4):
            box2[i,:] = box[i,0,:]
        corners = self.order_points(box2)

        # get the are of the contour
        area = cv2.contourArea(largest_contour)

        return corners, area

    def perspective_transform_corners(self,img,corners)

        dest_pts = np.array([(0, 0), (150,0), (150, 450), (0,450)])
        matrix = cv2.getPerspectiveTransform(np.float32(corners), np.float32(dest_pts))
        warped = cv2.warpPerspective(img, matrix, (150,450))
        plate_img.warped = warped.copy()
        
        plate_output = warped[300:400,:]
        cv2.imshow('w', warped)
        cv2.imshow('plate', plate_output)
        cv2.waitKey(1)

        filename = plate_img.base_file_name + '_w.png'
        os.chdir('/home/fizzer/Downloads/img_spam')
        cv2.imwrite(filename, warped)
        cv2.waitKey(1)



    def perspective_transform_plate(self, plate_img):
    
        test_img_hsv = cv2.cvtColor(plate_img.cropped, cv2.COLOR_BGR2HSV)


        lower1 = np.array([0,0,90])
        upper1 = np.array([0,10,210])
        mask1 = cv2.inRange(test_img_hsv,lower1,upper1)
        # lower2 = np.array([90,1,70])
        # upper2 = np.array([120,60,180])
        # mask2 = cv2.inRange(test_img_hsv,lower2,upper2)
        # hsv_mask = cv2.bitwise_or(mask1,mask2)
        hsv_mask = mask1


        hsv_plate = cv2.bitwise_and(test_img_hsv, test_img_hsv, hsv_mask)

        contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        longest_contours = []
        for c in contours:
            if len(c) > 10:
                longest_contours.append(c)

        cnt = sorted(contours, key=cv2.contourArea, reverse=True)[0]
            
        edges_img = np.zeros((test_img_hsv.shape[0], test_img_hsv.shape[1], 1),  dtype=np.uint8)

        cv2.drawContours(edges_img, longest_contours, -1, (255,255,255), 1)
        plate_img.edges = edges_img.copy()

        cv2.drawContours(hsv_plate, longest_contours, -1, (255,255,255), 1)
        # cv2.imshow('m', hsv_mask)
        # cv2.imshow('c', hsv_plate)
        # cv2.imshow('e', edges_img)

        edges_large = np.zeros((test_img_hsv.shape[0], test_img_hsv.shape[1], 1),  dtype=np.uint8)
        cv2.drawContours(edges_large, cnt, -1, (255, 255, 255), 1)
        cv2.imshow('edges large', edges_large)

        epsilon = 0.05*cv2.arcLength(cnt,True)
        box = cv2.approxPolyDP(cnt, epsilon, True)

        edges_box = np.zeros((test_img_hsv.shape[0], test_img_hsv.shape[1], 1),  dtype=np.uint8)
        cv2.drawContours(edges_box, box, -1, (255, 255, 255), 1)
        # cv2.imshow('edges box', edges_box)

        box2 = np.zeros((4,2))
        for i in range(4):
            box2[i,:] = box[i,0,:]
        ordered_box = self.order_points(box2)
            

        dest_pts = np.array([(0, 0), (150,0), (150, 450), (0,450)])
        # matrix = cv2.getPerspectiveTransform(np.float32(corners), np.float32(dest_pts))
        matrix = cv2.getPerspectiveTransform(np.float32(ordered_box), np.float32(dest_pts))
        warped = cv2.warpPerspective(plate_img.cropped, matrix, (150,450))
        plate_img.warped = warped.copy()
        
        plate_output = warped[300:400,:]
        cv2.imshow('w', warped)
        cv2.imshow('plate', plate_output)
        # cv2.imshow('h2', hsv_plate)
        cv2.waitKey(1)

        filename = plate_img.base_file_name + '_w.png'
        os.chdir('/home/fizzer/Downloads/img_spam')
        cv2.imwrite(filename, warped)
        cv2.waitKey(1)



    def order_points(self,pts):
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect 


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