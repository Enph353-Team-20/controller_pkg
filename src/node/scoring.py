#! /usr/bin/env python3

import roslib
import sys
import rospy
from std_msgs.msg import String
import time

from utils.neural_network import NeuralNetManager

def main(args):
    pub = rospy.Publisher("/license_plate", String)
    nn_manager = NeuralNetManager()
    rospy.init_node('scoring_manager', anonymous=True)
    rate = rospy.Rate(10)
    try:
        msg = str('Team20,silvertip,0,AA00')
        pub.publish(msg)
        start_time = time.time()

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

        
    msg = str('Team20,silvertip,-1,AA00')
    pub.publish(msg)

if __name__ == "__main__":
    main(sys.argv)