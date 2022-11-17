#! /usr/bin/env python3

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32


# define constants
# TODO update constants
xWidth = 800
centroidX = 300
corrVal = 50.0


# create class to run the camera/movement loop
class image_converter:

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
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # create the move object
        move = Twist()

        move.angular.z = 3

        # publish the move object
        try:
            self.cmd_pub.publish(move)
        except CvBridgeError as e:
            print(e)



# call the main function to run the class
def main(args):
    ic = image_converter()
    rospy.init_node('PID_motion', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)