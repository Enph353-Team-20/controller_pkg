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
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

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


### Stuff to make camera work

# def callback(data):
#     """Analyzes robot camera image to determine location of the line on the ground.
#     Args:
#         data (Image): Image pulled from camera topic
#     """
#     try:
#       cv_image = bridge.imgmsg_to_cv2(data, "mono8")
#     except CvBridgeError as e:
#       print(e)
#     cv2.imshow("Image window", cv_image)
#     cv2.waitKey(3)

# def main(args):
#     image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,callback)
#     rospy.init_node('camera_test', anonymous=True)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)






### Stuff to move robot

# rospy.init_node('pid_node')
# pub = rospy.Publisher('/R1/cmd_vel', Twist, 
#     queue_size=1)

# def updateCentroid(data):
#   """
#   Pulls location of the line from centroid topic, and calculates new driving directions
#   """
# #   centroidX = float(data.data)
# #   move = Twist()
# #   if (centroidX > 10 and centroidX < (xWidth - 10)):
# #     move.linear.x = 1
# #     move.angular.z = (xWidth/2 - centroidX)/corrVal
# #   else:
# #     move.linear.x = 0.5
# #     move.angular.z = 1
# #   pub.publish(move)

# #    move = Twist()
# #    move.linear.x = 1
# #    move.angular.z = 0.3
# #    pub.publish(move)


# def updateCorrVal(data):
#     """Pulls correction constant from corrVal topic. Only applicable during runtime.
#     Args:
#         data (std_msgs.msgs/Float32): New value
#     """
#     if (float(data.data) != 0):
#         corrVal = float(data.data)


# # sub = rospy.Subscriber('line_x', Int16, updateCentroid)
# # sub = rospy.Subscriber('corrVal', Float32, updateCorrVal)
# rate = rospy.Rate(2)

# while not rospy.is_shutdown():
#     move = Twist()
#     # move.linear.x = 1
#     move.angular.z = 0.3
#     pub.publish(move)
#     rate.sleep()