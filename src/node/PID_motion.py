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


# start time variable
start_time = time.time()

# define constants
threshold_val = 150


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
        # # publish license plates
        # self.plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)

    # the loop that will read from the camera and get the robot to move
    def callback(self,data):
        """Analyzes robot camera image to determine location of the line on the ground.
        Args:
            data (Image): Image pulled from camera topic
        """
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)


        # masking the hsv
        lower = np.array([0,0,100])
        upper = np.array([0,0,255])
        mask = cv2.inRange(hsv, lower, upper)

        # trying something out here
        # cv_image = cv_colour[:,:,0]

        # testing for threshold
        ret, thresh_img = cv2.threshold(cv_image, threshold_val, 255, cv2.THRESH_BINARY_INV)

        # get dimensions of the image
        image_height = cv_image.shape[0]
        image_width = cv_image.shape[1]

        # variable for which height to partition the screen into
        look_height = int(0.8*image_height)
        above_look = int(0.7*image_height)
        bottom_look = int(1*image_height)


        # find the center of the line
        cXR = self.findLineCentroidX(cv_image, (look_height, bottom_look),(int(image_width/2)-200,int(image_width)-1))
        cXL = self.findLineCentroidX(cv_image, (look_height, bottom_look), (200,int(image_width/2)))
        if cXR == -1:
            cXR = self.cXR_last
        else:
            self.cXR_last = cXR
        if cXL == -1:
            cXR = self.cXL_last
        else:
            self.cXL_last = cXL

        # find the center of the line for the screen above
        cXRa = self.findLineCentroidX(cv_image, (above_look, look_height),(int(image_width/2),int(image_width)-1-200))
        cXLa = self.findLineCentroidX(cv_image, (above_look, look_height), (200,int(image_width/2)))
        if cXRa == -1:
            cXRa = 640
        else:
            self.cXRa_last = cXRa
        if cXLa == -1:
            cXLa = 640
        else:
            self.cXLa_last = cXLa

        # find the slope
        slopeR = cXRa-cXR
        slopeL = cXLa-cXL


        # make a circle on the center of the image
        cv2.circle(thresh_img, (cXR, 300), 16, (0,255,0), -1)
        cv2.circle(thresh_img, (cXL, 300), 16, (0,255,0), -1)
        cv2.circle(thresh_img, (int((cXR+cXL)/2), 300), 25, (0,255,0), -1)

        # create the move object
        move = Twist()

        # attempt at a PID
        angMax = 4
        move.linear.x = 0.2

        # move.linear.x = 0
        # move.angular.z = 0 # for reseting the robot

        slope_turn_constant = 80

        if slopeL < slope_turn_constant and slopeR < -slope_turn_constant:
            # left line go bye bye (steer with right)
            move.angular.z = 2*(1-4*(cXR-640)/image_width)*angMax
            cv2.putText(img=thresh_img, text="Right", org=(1200, 400), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(0, 180, 0),thickness=1)
        elif slopeL > slope_turn_constant and slopeR > -slope_turn_constant:
            # right line go bye bye (steer with left)
            move.angular.z = 2*(1-4*(cXL)/image_width)*angMax
            cv2.putText(img=thresh_img, text="Left", org=(20, 400), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(0, 180, 0),thickness=1)
        else:
            # both lines in sight
            cX = int((cXR+cXL)/2)
            move.angular.z = (1-2*cX/image_width)*angMax
            cv2.putText(img=thresh_img, text="Both", org=(600, 400), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(0, 180, 0),thickness=1)



        # # make sure we turn left
        # current_time = time.time()
        # if current_time - start_time < 10:
        #     move.angular.z = (1-4*(cXL)/image_width)*angMax

        
        

        # publish the move object
        try:
            self.cmd_pub.publish(move)
        except CvBridgeError as e:
            print(e)

        # blur image to reduce noise
        thresh_img = cv2.GaussianBlur(thresh_img,(5,5),0)

        # channel = 0
        # ret_col, thresh_img_col = cv2.threshold(cv_colour[:,:,channel], threshold_val, 255, cv2.THRESH_BINARY_INV)

        # cv2.imshow("Image window", thresh_img)
        cv2.imshow("Image window", mask)
        # cv2.imshow("Image window", thresh_img_col)
        cv2.waitKey(1)

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
        # img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        cropped_img = img[y_range[0]:y_range[1],x_range[0]:x_range[1],:]


        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        # masking the hsv
        lower = np.array([0,0,100])
        upper = np.array([0,0,255])
        thresh_img = cv2.inRange(hsv_img, lower, upper)



        # ret, thresh_img = cv2.threshold(cropped_img, threshold_val, 255, cv2.THRESH_BINARY)

        # blur image to reduce noise
        thresh_img = cv2.GaussianBlur(thresh_img,(5,5),0)

        M = cv2.moments(thresh_img)
        try:
            cX = int(M["m10"] / M["m00"])
        except ZeroDivisionError:
            cX = -1

        return cX + x_range[0]-40
        



# call the main function to run the class
def main(args):
    rospy.init_node('PID_motion', anonymous=True)
    ic = image_converter()
    # ic.plate_pub.publish(test_plate)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)