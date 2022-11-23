#! /usr/bin/env python3

import roslib
import sys
import rospy
from std_msgs.msg import String
import time


def main(args):
    pub = rospy.Publisher("/license_plate", String)
    rospy.init_node('scoring_manager', anonymous=True)
    rate = rospy.Rate(10)
    try:
        msg = str('Team20,silvertip,0,AA00')
        pub.publish(msg)
        start_time = time.time()

        while (time.time() < start_time + 30.0):
            pass
        
        msg = str('Team20,silvertip,-1,AA00')
        pub.publish(msg)

    except rospy.ROSInterruptException:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)