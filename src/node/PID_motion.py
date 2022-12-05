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

    last_time = time.time()

    centerR_last = (640,719)
    centerL_last = (640,719)
    centAbR_last = (640,719)
    centAbL_last = (640,719)

    # initiate the class
    def __init__(self):
        self.bridge = CvBridge()
        # subscribe to camera
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        # publish to movement
        self.cmd_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)

        # state variable
        self.state = "turn left"
        self.in_crosswalk = False

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

        # create the move object
        move = Twist()

        # get dimensions of the image
        image_height = cv_image.shape[0]
        image_width = cv_image.shape[1]

        # masking the hsv
        lower = np.array([0,0,white_threshold])
        upper = np.array([0,0,255])
        thresh_img = cv2.inRange(hsv, lower, upper)

        # # include for binary threshold
        # ret, thresh_img = cv2.threshold(cv_image, threshold_val, 255, cv2.THRESH_BINARY_INV)

        # variable for which height to partition the screen into
        look_height = int(0.8*image_height)
        above_look = int(0.7*image_height)
        bottom_look = int(1*image_height)

        # masking the hsv to filter for white lines
        lower = np.array([0,0,white_threshold])
        upper = np.array([0,0,255])
        line_mask = cv2.inRange(hsv, lower, upper)

        # ideal slope of the lines to drive straight
        slope = 0.225

        # troubleshooting state
        if self.state == "do nothing":
            move.linear.x = 0
            move.angular.z = 0









        # start of state machine

        if self.state == "turn left":

            # masking the hsv to filter for white lines
            lower = np.array([0,0,white_threshold])
            upper = np.array([0,0,255])
            line_mask = cv2.inRange(hsv, lower, upper)

            # getting the blue mask
            low_blue = np.array([115, 50, 50])
            high_blue = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv, low_blue, high_blue)

            # get left line centroid
            centersL = self.getCentroid(line_mask,(look_height, bottom_look), (0,int(image_width/2)))

            # get blue centroid
            blue_cent = self.getCentroid(blue_mask,(look_height, bottom_look), (0,image_width-1))

            # calculate ideal center of left line
            eCL = (int(slope*centersL[1] + 95), centersL[1])

            # plot centroid on troubleshooting screen
            cv2.circle(thresh_img, centersL, 16, (255,255,255), -1)

            # plot ideal centroid on screen
            # cv2.circle(line_mask, eCL, 16, (255,255,255), -1)

            # define driving constants
            angMax = 5
            maxSpeed = 0.6
            turnReduction = 0.2

            # drive left
            move.angular.z = self.PIDcontrol(centersL[0]-eCL[0],angMax,image_width/2)
            move.linear.x = max(maxSpeed - turnReduction*abs(move.angular.z),0)

            # switch states after some time (comment in if the below if statement stops working)
            # if time.time() > self.start_time + 2:
            #     self.state = "drive"

            if blue_cent[1] > 0 and blue_cent[0] > image_width/2:
                self.state = "drive"
                







        elif self.state == "drive":

            # masking the hsv to filter for white lines
            lower = np.array([0,0,white_threshold])
            upper = np.array([0,0,255])
            line_mask = cv2.inRange(hsv, lower, upper)

            # compute centroids
            centersR = self.getCentroid(line_mask,(look_height, bottom_look),(int(image_width/2),int(image_width)-1))
            centersL = self.getCentroid(line_mask,(look_height, bottom_look), (0,int(image_width/2)))
            if centersR[0] == -1:
                centersR = self.centerR_last
            else:
                self.centerR_last = centersR
            if centersL[0] == -1:
                centersL = self.centerL_last
            else:
                self.centerL_last = centersL
            centersC = (int((centersR[0]+centersL[0])/2), int((centersR[1]+centersL[1])/2))

            # compute above centroids
            centAbR = self.getCentroid(line_mask,(above_look,look_height),(int(image_width/2),int(image_width)-1))
            centAbL = self.getCentroid(line_mask,(above_look,look_height), (0,int(image_width/2)))
            if centAbR[0] == -1:
                centAbR = self.centAbR_last
            else:
                self.centAbR_last = centAbR
            if centAbL[0] == -1:
                centAbL = self.centAbL_last
            else:
                self.centAbL_last = centAbL

            # compute ideal centers
            eCR = (int(-slope*centersR[1] + 1185), centersR[1])
            eCL = (int(slope*centersL[1] + 95), centersL[1])

            # plot centroids on the screen
            cv2.circle(thresh_img, centersR, 16, (255,255,255), -1)
            cv2.circle(thresh_img, centersL, 16, (255,255,255), -1)
            cv2.circle(thresh_img, centersC, 25, (255,255,255), -1)

            # plot ideal centroids on screen
            # cv2.circle(thresh_img, eCR, 16, (255,255,255), -1)
            # cv2.circle(thresh_img, eCL, 16, (255,255,255), -1)

            # define how steep a change is required for a turn to be detected
            turn_diff = 20

            # find relative differences between above and below screens
            delta_xR = centersR[0] - centAbR[0]
            delta_xL = centAbL[0] - centersL[0]

            # define driving constants
            angMax = 4
            maxSpeed = 0.5
            turnReduction = 0.4

            # determine which line to follow and drive off that line
            if delta_xL < turn_diff and delta_xR > turn_diff:
                # drive right
                move.angular.z = self.PIDcontrol(centersR[0]-eCR[0],angMax,image_width/2)
                cv2.putText(img=thresh_img, text="Right", org=(1200, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            elif delta_xR < turn_diff and delta_xL > turn_diff:
                # drive left
                move.angular.z = self.PIDcontrol(centersL[0]-eCL[0],angMax,image_width/2)
                cv2.putText(img=thresh_img, text="Left", org=(20, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            else:
                # drive with both
                move.angular.z = self.PIDcontrol(centersC[0]-image_width/2,angMax,image_width)
                cv2.putText(img=thresh_img, text="Both", org=(600, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)

            # determine linear speed based on how much we're truning
            move.linear.x = max(maxSpeed - turnReduction*abs(move.angular.z),-0.05)

            # filter for the red line
            red_lower = np.array([0,200,0])
            red_upper = np.array([6,255,255])
            red_mask = cv2.inRange(hsv,red_lower,red_upper)

            # get the red center
            red_center = self.getCentroid(red_mask,(0,image_height-1),(0,image_width-1))

            # see if it has made a loop
            # if self.pedestrian_count == 2 and time.time() > self.detection_time + 3:
            if self.pedestrian_count == 2 and self.in_crosswalk == False:
                if red_center[1] > 375 and abs(red_center[0] - image_width/2) < 200:
                    self.state = "turn inside"
                    print("turn to inner")

            if red_center[1] < 300:
                self.in_crosswalk = False

            # stop if the crosswalk is there
            if red_center[1] > 550:
                if self.in_crosswalk == False:
                    self.in_crosswalk = True
                    self.state = "crosswalk"
                    self.delay_time = time.time()





        elif self.state == "turn inside":

            # masking the hsv to filter for white lines
            lower = np.array([0,0,white_threshold])
            upper = np.array([0,0,255])
            line_mask = cv2.inRange(hsv, lower, upper)

            # getting the blue mask
            low_blue = np.array([115, 50, 50])
            high_blue = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv, low_blue, high_blue)

            # get left line centroid
            centersL = self.getCentroid(line_mask,(look_height, bottom_look), (0,int(image_width/2)))

            # get blue centroid
            blue_cent = self.getCentroid(blue_mask,(0, bottom_look), (0,image_width-1))

            # calculate ideal center of left line
            eCL = (int(slope*centersL[1] + 95), centersL[1])

            # plot centroid on troubleshooting screen
            cv2.circle(thresh_img, centersL, 16, (255,255,255), -1)

            # plot ideal centroid on screen
            # cv2.circle(line_mask, eCL, 16, (255,255,255), -1)

            # define driving constants
            angMax = 5
            maxSpeed = 0.6
            turnReduction = 0.2

            # drive left
            move.angular.z = self.PIDcontrol(centersL[0]-eCL[0],angMax,image_width/2)
            move.linear.x = max(maxSpeed - turnReduction*abs(move.angular.z),0)


            # filter for the red line
            red_lower = np.array([0,200,0])
            red_upper = np.array([6,255,255])
            red_mask = cv2.inRange(hsv,red_lower,red_upper)

            # get the red center
            red_center = self.getCentroid(red_mask,(0,image_height-1),(0,image_width-1))

            if red_center[0] == -1:
                self.state = "car"
                self.delay_time = time.time()







        elif self.state == "inner loop":

            # masking the hsv to filter for white lines
            lower = np.array([0,0,150])
            upper = np.array([0,0,255])
            line_mask = cv2.inRange(hsv, lower, upper)

            # masking the hsv to filter for the road
            lower_road = np.array([0,0,80])
            upper_road = np.array([0,0,100])
            road_mask = cv2.inRange(hsv, lower_road, upper_road)

            # get the center of the road
            cent_road = self.getCentroid(road_mask,(look_height,bottom_look), (0,image_width-1))

            # masking for blue
            low_blue = np.array([115, 50, 50])
            high_blue = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv, low_blue, high_blue)

            # blue center
            blue_cent = self.getCentroid(blue_mask,(look_height,bottom_look),(0,image_width-1))
            if blue_cent[0] == -1:
                blue_cent = (0,719)

            # compute centroids
            centersR = self.getCentroid(line_mask,(look_height, bottom_look),(int(image_width/2),int(image_width)-1))
            centersL = self.getCentroid(line_mask,(look_height, bottom_look), (0,int(image_width/2)))
            if centersR[0] == -1:
                centersR = self.centerR_last
            else:
                self.centerR_last = centersR
            if centersL[0] == -1:
                centersL = self.centerL_last
            else:
                self.centerL_last = centersL
            centersC = (int((centersR[0]+centersL[0])/2), int((centersR[1]+centersL[1])/2))

            # compute ideal centers
            eCR = (int(-slope*centersR[1] + 1185), centersR[1])
            eCL = (int(slope*centersL[1] + 95), centersL[1])

            # plot centroids on the screen
            cv2.circle(thresh_img, centersR, 16, (255,255,255), -1)
            cv2.circle(thresh_img, centersL, 16, (255,255,255), -1)
            cv2.circle(thresh_img, centersC, 25, (255,255,255), -1)

            # define driving constants
            angMax = 5
            maxSpeed = 0.45
            turnReduction = 0.2

            # turn left after seeing the car
            if time.time() > self.detection_time + 1 and time.time() < self.detection_time + 5:
                # drive left
                move.angular.z = self.PIDcontrol(centersL[0]-eCL[0],angMax,image_width/2)
                move.linear.x = max(maxSpeed - 0.5*turnReduction*abs(move.angular.z),0)
            # drive the inner loop
            elif time.time() > self.detection_time + 5:
                road_turn = self.PIDcontrol(cent_road[0]-image_width/2-80,angMax,image_width)
                blue_turn = self.PIDcontrol(blue_cent[0]-image_width/2,0.3*angMax,image_width)
                move.angular.z = road_turn-blue_turn
                move.linear.x = max(maxSpeed - turnReduction*abs(move.angular.z),-0.05)
            # wait for the car to pass by
            else:
                move.linear.x = 0
                move.angular.z = 0






        elif self.state == "crosswalk":
            
            # stop the car
            move.angular.z = 0
            move.linear.x = 0

            # look for the pedestrian
            error_val = 0.5
            img_val = self.averageImageValue(gray,640,360,100)
            if time.time()>self.delay_time+0.5:
                if abs(img_val-self.img_val_last) > error_val or time.time() > self.delay_time+6:
                    print("pedestrian")
                    self.pedestrian_count += 1
                    cv2.circle(thresh_img, (640, 400), 49, (255,255,255), -1)
                    self.state = "drive"
                    self.detection_time = time.time()
            self.img_val_last = img_val







        elif self.state == "car":

            # stop the car
            move.angular.z = 0
            move.linear.x = 0

            # look for the car
            error_val = 1
            img_val = self.averageImageValue(gray,300,360,100)
            if time.time()>self.delay_time+0.5:
                if img_val > self.img_val_last+error_val or img_val<self.img_val_last-error_val:
                    print("car")
                    cv2.circle(thresh_img, (640, 400), 49, (255,255,255), -1)
                    self.state = "inner loop"
                    self.pedestrian_count += 1
                    self.detection_time = time.time()
            self.img_val_last = img_val


        
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






    def getCentroid(self, img, y_range, x_range):

        # crop to look at desired window
        cropped_img = img[y_range[0]:y_range[1],x_range[0]:x_range[1]]

        # get moments
        M = cv2.moments(cropped_img)
        try:
            cX = int(M["m10"] / M["m00"]) + x_range[0]
        except ZeroDivisionError:
            cX = -1
        try:
            cY = int(M["m01"]/M["m00"]) + y_range[0]
        except ZeroDivisionError:
            cY = -1

        return (cX,cY)

    def PIDcontrol(self, error, max_turn, frame_width):
        return -max_turn*error/(0.5*frame_width)



    def averageImageValue(self, img, X, Y, size):

        cropped_img = img[Y-size:Y+size,X-size:X+size]

        # blur image to reduce noise
        cropped_img = cv2.GaussianBlur(cropped_img,(1,1),0)

        return np.mean(cropped_img)



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