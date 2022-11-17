
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

bridge = CvBridge()
template_img = cv2.imread("/home/fizzer/ros_ws/src/controller_pkg/src/node/media/P01.png", cv2.IMREAD_GRAYSCALE)
print(template_img.shape)
# cv2.imshow("Template", template_img)
# cv2.waitKey(0)

def callback(data):
    """Analyzes robot camera image to determine location of the line on the ground.
    Args:
        data (Image): Image pulled from camera topic
    """
    
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
        print(e)


    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(2)

    found_plate = sift_locate(template_img, cv_image)
    cv2.imshow("Image window", found_plate)
    cv2.waitKey(1)

def sift_locate(template, frame):
    # Get Features in image
    # img = cv2.imread(self.template_path, cv2.IMREAD_GRAYSCALE)
    sift = cv2.SIFT_create()
    kp_image, desc_image = sift.detectAndCompute(template, None)
    # Draw keypoints on image
    template_kp = cv2.drawKeypoints(template, kp_image, template)
    # cv2.imshow("Template", template_kp)

    #Start feature matching
    index_params = dict(algorithm=0, trees=5)
    search_params = dict()
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Get keypoints in camera frame
    grayframe = frame
    # ret, grayframe = cv2.threshold(frame, 15, 255, cv2.THRESH_BINARY)    
    kp_grayframe, desc_grayframe = sift.detectAndCompute(grayframe, None)

    # frame = cv2.drawKeypoints(grayframe, kp_grayframe, grayframe)

    # cv2.imshow("Image window 2", grayframe)
    # cv2.waitKey(1)
    
    matches = flann.knnMatch(desc_image, desc_grayframe,k=2)

    good_points = []
    # Ratio test. Lower better
    for m,n in matches: #m: query images, n: from grayframe
        if m.distance < 0.6*n.distance:
            good_points.append(m)

    # debug_frame = cv2.drawMatches(template_img, kp_image, grayframe, kp_grayframe, good_points, grayframe)
    # cv2.imshow("Debug", debug_frame)

    if len(good_points) > 3:
        # Extract position of points and reformat array
        query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)

        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
        matches_mask = mask.ravel().tolist()

        # Perspective transf,orm
        s = template.shape
        pts = np.float32([[0, 0], [0, s[0]], [s[1], s[0]], [s[1], 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, matrix)

        frame = cv2.polylines(frame, [np.int32(dst)], True, (255, 0, 0), 3)

    if (len(good_points) > 0):
        grayframe_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points])
        ctrd = centroid_of_points(grayframe_pts)
        cv2.circle(frame, (int(ctrd[0]), int(ctrd[1])), 7, (0,255,0), -1)

    return frame

def centroid_of_points(points):
    avg_x = sum([pt[0] for pt in points])/len(points)
    avg_y = sum([pt[1] for pt in points])/len(points)
    return (avg_x, avg_y)


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