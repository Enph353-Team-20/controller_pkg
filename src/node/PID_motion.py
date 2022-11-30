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
white_threshold = 100


# create class to run the camera/movement loop
class image_converter:

    cXR_last = 640
    cXL_last = 640
    cXRa_last = 640
    cXLa_last = 640
    img_val_last = 97.8
    detection_time = 0
    start_time = time.time()
    delay_time = 0
    pedestrian_count = 0

    # initiate the class
    def __init__(self):
        self.bridge = CvBridge()
        # subscribe to camera
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        # publish to movement
        self.cmd_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)

        # state variable
        self.state = "turn left"
        self.crosswalk = False
        self.pedestrian = False

    # the loop that will read from the camera and get the robot to move
    def callback(self,data):
        """Analyzes robot camera image to determine location of the line on the ground.
        Args:
            data (Image): Image pulled from camera topic
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            gray = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)


        # masking the hsv
        lower = np.array([0,0,white_threshold])
        upper = np.array([0,0,255])
        thresh_img = cv2.inRange(hsv, lower, upper)

        # # include for binary threshold
        # ret, thresh_img = cv2.threshold(cv_image, threshold_val, 255, cv2.THRESH_BINARY_INV)

        # get dimensions of the image
        image_height = cv_image.shape[0]
        image_width = cv_image.shape[1]

        # variable for which height to partition the screen into
        look_height = int(0.8*image_height)
        above_look = int(0.7*image_height)
        bottom_look = int(1*image_height)

        # create the move object
        move = Twist()

        # PID turning values and speed
        angMax = 4
        move.linear.x = 0.2

        lower_car = np.array([0,0,50])
        upper_car = np.array([360,0,75])

        # lower_car = np.array([0,30,0])
        # upper_car = np.array([12,254,254])

        car_thresh = cv2.inRange(hsv, lower_car, upper_car)





        # state machine start

        if self.state == "turn left":

            # find left centroid
            cXL = self.findLineCentroidX(cv_image, (look_height, bottom_look), (200,int(image_width/2)))
            if cXL == -1:
                cXL = self.cXL_last
            else:
                self.cXL_last = cXL

            # PID steering with left line
            move.angular.z = 2*(1-4*(cXL)/image_width)*angMax

            if time.time() > self.start_time + 3:
                self.state = "drive"

        elif self.state == "drive":

            # find the center of the line
            cXR = self.findLineCentroidX(cv_image, (look_height, bottom_look),(int(image_width/2)-200,int(image_width)-1))
            cXL = self.findLineCentroidX(cv_image, (look_height, bottom_look), (200,int(image_width/2)))
            if cXR == -1:
                cXR = self.cXR_last
            else:
                self.cXR_last = cXR
            if cXL == -1:
                cXL = self.cXL_last
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

            # find the difference between centroids in the upper and lower windows
            slopeR = cXRa-cXR
            slopeL = cXLa-cXL

            # plot left and right centroids on the image
            cv2.circle(thresh_img, (cXR, 300), 16, (255,255,255), -1)
            cv2.circle(thresh_img, (cXL, 300), 16, (255,255,255), -1)
            cv2.circle(thresh_img, (int((cXR+cXL)/2), 300), 25, (255,255,255), -1)

            # constant to determine which line to follow (big impact on PID performance)
            slope_turn_constant = 30

            # determine which line to follow with PID
            if slopeL < slope_turn_constant and slopeR < -slope_turn_constant:
                # left line go bye bye (steer with right)
                move.angular.z = 2*(1-4*(cXR-640)/image_width)*angMax
                cv2.putText(img=thresh_img, text="Right", org=(1200, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            elif slopeL > slope_turn_constant and slopeR > -slope_turn_constant:
                # right line go bye bye (steer with left)
                move.angular.z = 2*(1-4*(cXL)/image_width)*angMax
                cv2.putText(img=thresh_img, text="Left", org=(20, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            else:
                # both lines in sight
                cX = int((cXR+cXL)/2)
                move.angular.z = (1-2*cX/image_width)*angMax
                cv2.putText(img=thresh_img, text="Both", org=(600, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)

            # filter for red
            red_lower = np.array([0,200,0])
            red_upper = np.array([6,255,255])
            red_mask = cv2.inRange(hsv,red_lower,red_upper)

            # see if it has made a loop
            if self.pedestrian_count > 1 and time.time() > self.detection_time+9:
                self.state = "turn to inner"
                self.delay_time = time.time()

            # stop if the crosswalk is there
            red_center = self.findCentroidY(red_mask)
            if red_center > 550:
                if time.time() > self.detection_time+3:
                    self.state = "crosswalk"
                    self.delay_time = time.time()


        elif self.state == "crosswalk":
            
            # stop the car
            move.angular.z = 0
            move.linear.x = 0

            # look for the pedestrian
            error_val = 0.5
            img_val = self.averageImageValue(gray,640,360,100)
            if time.time()>self.delay_time+1:
                if img_val > self.img_val_last+error_val or img_val<self.img_val_last-error_val:
                    print("pedestrian")
                    self.pedestrian_count += 1
                    cv2.circle(thresh_img, (640, 400), 49, (255,255,255), -1)
                    self.state = "drive"
                    self.detection_time = time.time()
            self.img_val_last = img_val

        elif self.state == "turn to inner":

            # find left centroid
            cXL = self.findLineCentroidX(cv_image, (look_height, bottom_look), (200,int(image_width/2)))
            if cXL == -1:
                cXL = self.cXL_last
            else:
                self.cXL_last = cXL

            # PID steering with left line
            move.angular.z = 2*(1-4*(cXL)/image_width)*angMax

            # drive long enough to make a turn
            if time.time() > self.delay_time + 3.5:
                self.delay_time = time.time()
                self.state = "car"

        elif self.state == "car":

            # stop the car
            move.angular.z = 0
            move.linear.x = 0

            # look for the car
            error_val = 0.5
            img_val = self.averageImageValue(gray,640,360,100)
            if time.time()>self.delay_time+1:
                if img_val > self.img_val_last+error_val or img_val<self.img_val_last-error_val:
                    print("car")
                    cv2.circle(thresh_img, (640, 400), 49, (255,255,255), -1)
                    self.state = "inner loop"
                    self.detection_time = time.time()
            self.img_val_last = img_val

        elif self.state == "inner loop":

            # find the center of the line
            cXR = self.findLineCentroidX(cv_image, (look_height, bottom_look),(int(image_width/2)-200,int(image_width)-1))
            cXL = self.findLineCentroidX(cv_image, (look_height, bottom_look), (200,int(image_width/2)))
            cXc = self.findCarCenter(car_thresh,(look_height, bottom_look), (int(image_width/2)-600,int(image_width/2)+600))
            if cXR == -1:
                cXR = self.cXR_last
            else:
                self.cXR_last = cXR
            if cXL == -1:
                cXL = self.cXL_last
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

            # find the difference between centroids in the upper and lower windows
            slopeR = cXRa-cXR
            slopeL = cXLa-cXL

            # plot left and right centroids on the image
            cv2.circle(thresh_img, (cXR, 300), 16, (255,255,255), -1)
            cv2.circle(thresh_img, (cXL, 300), 16, (255,255,255), -1)
            cv2.circle(thresh_img, (int((cXR+cXL)/2), 300), 25, (255,255,255), -1)

            # constant to determine which line to follow (big impact on PID performance)
            slope_turn_constant = 10000

            # start driving a little after it sees the car so it doesn't run into it
            if time.time() > self.detection_time + 1:
                # determine which line to follow with PID
                if slopeL > slope_turn_constant and slopeR > -slope_turn_constant:
                    # right line go bye bye (steer with left)
                    move.angular.z = 2*(1-4*(cXL)/image_width)*angMax
                    cv2.putText(img=thresh_img, text="Left", org=(20, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
                elif slopeL < slope_turn_constant and slopeR < -slope_turn_constant:
                    # left line go bye bye (steer with right)
                    move.angular.z = 2*(1-4*(cXR-640)/image_width)*angMax
                    cv2.putText(img=thresh_img, text="Right", org=(1200, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
                else:
                    # follow the car
                    move.linear.x = 0.3
                    move.angular.z = (1-2*cXc/image_width)*angMax
                    cv2.putText(img=thresh_img, text="Car", org=(600, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            else:
                move.linear.x = 0
                move.angular.z = 0

        
        
        
        # publish the move object
        try:
            self.cmd_pub.publish(move)
        except CvBridgeError as e:
            print(e)

        # print current state to the screen
        cv2.putText(img=thresh_img, text=self.state, org=(600, 50), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)

        # display troubleshooting screen
        cv2.imshow("Image window", thresh_img)
        cv2.waitKey(1)


    def findLineCentroidX(self, img, y_range, x_range):
        """Finds X-coordinate centroid of the line
        Checks pixels across entire width of image and between y values defined in y_range
        Args:
            img (Image): Image to analyze
            y_range (Tuple(int, int)): Y range of pixels to search.
            x_raneg (Tuple(int, int)): X range of pixels to search.
        Returns:
            int: Centroid of line
        """

        # blur image to reduce noise
        img = cv2.GaussianBlur(img,(5,5),0)

        # crop to look at desired window
        cropped_img = img[y_range[0]:y_range[1],x_range[0]:x_range[1],:]

        # convert image to HSV format
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # masking the hsv to filter for white lines
        lower = np.array([0,0,white_threshold])
        upper = np.array([0,0,255])
        thresh_img = cv2.inRange(hsv_img, lower, upper)

        # get moments
        M = cv2.moments(thresh_img)
        try:
            cX = int(M["m10"] / M["m00"])
        except ZeroDivisionError:
            cX = -1

        return cX + x_range[0]-40


    def findCentroidY(self, img):
        M = cv2.moments(img)
        try:
            cY = int(M["m01"] / M["m00"])
        except ZeroDivisionError:
            cY = -1

        return cY

    def averageImageValue(self, img, X, Y, size):

        cropped_img = img[Y-size:Y+size,X-size:X+size]

        # blur image to reduce noise
        cropped_img = cv2.GaussianBlur(cropped_img,(1,1),0)

        return np.mean(cropped_img)
        
    def findCarCenter(self, img, y_range, x_range):

        # crop to look at desired window
        cropped_img = img[y_range[0]:y_range[1],x_range[0]:x_range[1]]

        # get moments
        M = cv2.moments(cropped_img)
        try:
            cX = int(M["m10"] / M["m00"])
        except ZeroDivisionError:
            cX = -1

        return cX



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