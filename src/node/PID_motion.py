#! /usr/bin/env python3

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32


# define constants
# TODO update constants
xWidth = 800
centroidX = 300
corrVal = 50.0

threshold_val = 220


# create class to run the camera/movement loop
class image_converter:

    cXR_last = 640
    cXL_last = 640
    cXRa_last = 640
    cXLa_last = 640

    # initiate the class
    def __init__(self):
        self.bridge = CvBridge()
        # subscribe to camera
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        # publish to movement
        self.cmd_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)

    # the loop that will read from the camera and get the robot to move
    def callback(self,data):
        """Analyzes robot camera image to determine location of the line on the ground.
        Args:
            data (Image): Image pulled from camera topic
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)

        # blur image to reduce noise
        cv_image = cv2.GaussianBlur(cv_image,(3,3),10)

        # testing for threshold
        ret, thresh_img = cv2.threshold(cv_image, threshold_val, 255, cv2.THRESH_BINARY_INV)

        # get dimensions of the image
        image_height = cv_image.shape[0]
        image_width = cv_image.shape[1]

        # variable for which height to partition the screen into
        look_height = int(0.7*image_height)
        above_look = int(0.6*image_height)


        # find the center of the line
        cXR = self.findLineCentroidX(cv_image, (look_height, -1),(int(image_width/2),int(image_width)-1))
        cXL = self.findLineCentroidX(cv_image, (look_height, -1), (0,int(image_width/2)))
        if cXR == -1:
            cXR = self.cXR_last
        else:
            self.cXR_last = cXR
        if cXL == -1:
            cXL = self.cXL_last
        else:
            self.cXL_last = cXL

        # find the center of the line for the screen above
        cXRa = self.findLineCentroidX(cv_image, (above_look, look_height),(int(image_width/2),int(image_width)-1))
        cXLa = self.findLineCentroidX(cv_image, (above_look, look_height), (0,int(image_width/2)))
        if cXRa == -1:
            cXRa = self.cXRa_last
        else:
            self.cXRa_last = cXRa
        if cXLa == -1:
            cXLa = self.cXLa_last
        else:
            self.cXLa_last = cXLa

        # find the slope
        slopeR = cXRa-cXR
        slopeL = cXLa-cXL

        # make a circle on the center of the image
        cv2.circle(thresh_img, (int((cXR+cXL)/2), 700), 25, (0,255,0), -1)

        # create the move object
        move = Twist()

        # attempt at a PID
        angMax = 4
        move.linear.x = 0.2

        # move.linear.x = 0
        # move.angular.z = 0 # for reseting the robot

        slope_turn_constant = 10


        if slopeL < slope_turn_constant and slopeR < -slope_turn_constant:
            # left line go bye bye (steer with right)
            move.angular.z = (1-4*(cXR-640)/image_width)*angMax
        elif slopeL > slope_turn_constant and slopeR > -slope_turn_constant:
            # right line go bye bye (steer with left)
            move.angular.z = (1-4*cXL/image_width)*angMax
        else:
            # both lines in sight
            cX = int((cXR+cXL)/2)
            move.angular.z = (1-2*cX/image_width)*angMax

        
        

        # publish the move object
        try:
            self.cmd_pub.publish(move)
        except CvBridgeError as e:
            print(e)


        cv2.imshow("Image window", thresh_img)
        # cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    # function to find the center of the line
    def findLineCentroidX(self, img, y_range, x_range):
        """Finds X-coordinate centroid of the line
        Checks pixels across entire width of image and between y values defined in y_range
        Args:
            img (Image): Image to analyze
            y_range (Tuple(int, int)): Y range of pixels to search. 
        Returns:
            int: Centroid of line
        """
        cropped_img = img[y_range[0]:y_range[1],x_range[0]:x_range[1]]

        ret, thresh_img = cv2.threshold(cropped_img, threshold_val, 255, cv2.THRESH_BINARY)

        M = cv2.moments(thresh_img)
        try:
            cX = int(M["m10"] / M["m00"])
        except ZeroDivisionError:
            cX = -1

        return cX + x_range[0]
        



# call the main function to run the class
def main(args):
    rospy.init_node('PID_motion', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)