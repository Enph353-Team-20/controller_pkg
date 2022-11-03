#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

xWidth = 800
centroidX = 300
corrVal = 50.0

rospy.init_node('pid_node')
pub = rospy.Publisher('/R1/cmd_vel', Twist, 
    queue_size=1)

def updateCentroid(data):
  """
  Pulls location of the line from centroid topic, and calculates new driving directions
  """
#   centroidX = float(data.data)
#   move = Twist()
#   if (centroidX > 10 and centroidX < (xWidth - 10)):
#     move.linear.x = 1
#     move.angular.z = (xWidth/2 - centroidX)/corrVal
#   else:
#     move.linear.x = 0.5
#     move.angular.z = 1
#   pub.publish(move)

#    move = Twist()
#    move.linear.x = 1
#    move.angular.z = 0.3
#    pub.publish(move)


def updateCorrVal(data):
    """Pulls correction constant from corrVal topic. Only applicable during runtime.
    Args:
        data (std_msgs.msgs/Float32): New value
    """
    if (float(data.data) != 0):
        corrVal = float(data.data)


# sub = rospy.Subscriber('line_x', Int16, updateCentroid)
# sub = rospy.Subscriber('corrVal', Float32, updateCorrVal)
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    move = Twist()
    # move.linear.x = 1
    move.angular.z = 0.3
    pub.publish(move)
    rate.sleep()