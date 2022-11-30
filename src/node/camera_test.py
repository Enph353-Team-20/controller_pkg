
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

bridge = CvBridge()
prevTime = time.time()
ran_once = False

def callback(data):
    global prevTime
    global ran_once
    """Analyzes robot camera image to determine location of the line on the ground.
    Args:
        data (Image): Image pulled from camera topic
    """
    
    # try:
    #     cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #     print(e)
    # cv2.imshow('Raw', cv_image)
    # cv2.waitKey(1)
    
    # if (time.time() - prevTime < 5.0):
    #     scaled = cv2.resize(cv_image, (int(cv_image.shape[1]*0.5), int(cv_image.shape[0]*0.5)))
    #     os.chdir('/home/fizzer/Downloads/img_spam')
    #     cv2.imwrite('p' + str(int(time.time())) + '.png', scaled)
    #     prevTime = time.time()

    cv_image = cv2.imread("/home/fizzer/Downloads/img_spam/img1669675999_gp_4.png", cv2.IMREAD_UNCHANGED)
    
    test_img_hsv = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2HSV)
    denoised = cv2.fastNlMeansDenoising(test_img_hsv, h=5)

    hsv_mask = cv2.inRange(denoised, np.array((0, 0, 100)), np.array((0, 0, 210)))
    hsv_plate = cv2.bitwise_and(test_img_hsv, test_img_hsv, hsv_mask)

    contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    longest_contours = []
    for c in contours:
        if len(c) > 10:
            longest_contours.append(c)

    print(longest_contours)
        
    edges_img = np.zeros((cv_image.shape[0], cv_image.shape[1], 1),  dtype=np.uint8)

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
        if abs(l[0][3] - l[0][1]) <= 10 and abs(top_y - l[0][1]) <= 5: 
            topmost_line = l
        if abs(l[0][3] - l[0][1]) <= 10 and abs(bottom_y - l[0][1]) <= 5: 
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
    warped = cv2.warpPerspective(cv_image, matrix, (150,450))
    
    # cv2.drawContours(edges_img, longest_contours, -1, (255,255,255), 1)
    cv2.drawContours(hsv_plate, longest_contours, -1, (255,255,255), 1)
    # cv2.imshow('m', hsv_mask)
    # cv2.imshow('c', hsv_plate)
    # cv2.imshow('e', edges_img)
    cv2.imshow('w', warped)
    cv2.imshow('h2', hsv_plate)
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


    #"""Approach: contour max min"""
    # bottom_y = hsv_mask.shape[0]
    # top_y = 0
    # topmost_contour = None
    # bottommost_contour = None
    # for co in longest_contours:
    #     ys = co[:,0]
    #     co_maxy = np.amax(ys)
    #     co_miny = np.amin(ys)
    #     if co_maxy > top_y:
    #         top_y = co_maxy
    #         topmost_contour = co

    #     if co_miny < bottom_y:
    #         bottom_y = co_miny
    #         bottommost_contour = co
        
    # corners = []
    # corner1 = np.where(topmost_contour[1] == top_y)
    # corner3 = np.where(bottommost_contour[1] == bottom_y)
    # corners.append(corner1)
    # corners.append(corner3)

    # if not ran_once:
    #     ran_once = True
    #     print(top_y)
    #     print(topmost_contour)
    #     print(bottommost_contour)
    #     print(bottom_y)
    #     print(corners)

    # # print(longest_contours)
    # cv2.drawContours(hsv_plate, longest_contours, -1, (0,255,0), 3)
    # for c in corners:
    #     cv2.circle(hsv_plate, (c[0][0], c[0][1]), 5, (0,255,0), -1)