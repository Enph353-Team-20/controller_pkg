#!/usr/bin/env python3

import cv2
import numpy as np
import time

class SiftProcessor():

    def __init__(self, templates):
        """Initiate SiftProcessor.
        
        Args:
            templates (list(np.array(int))): Single-channel cv2 images
        """
        self.sift = cv2.SIFT_create()

        self.index_params = dict(algorithm=0, trees=5)
        self.search_params = dict()
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        self.templates = templates
        self.tmplt_kp, self.tmplt_desc = self.populate_tmplt_info()

        self.input = None # an image, in which each template to be searched.
        self.input_kp = [] 
        self.input_desc = []
        self.output = None # a list of 'good' keypoints that pass ratio test 
        self.debug_flags = {
            "template_kp": False,
            "input_kp": False,
            "homogr": False,
            "matches": False
        }
        self.debug_imgs = {
            "template_kp": None,
            "input_kp": None,
            "homogr": None,
            "matches": None
        }

    def populate_tmplt_info(self):
        kp = []
        desc = []
        for i in range(len(self.templates)):
            tmplt_kp, tmplt_desc = self.sift.detectAndCompute(self.templates[0], None)
            kp.append(tmplt_kp)
            desc.append(tmplt_desc)
        return kp, desc

    def input_image(self, image):
        """Queue an image for sift processing.

        Args:
            image (np.array(int)): Single-channel cv2 image
        """
        # image = cv2.resize(image, (int(image.shape[1]*0.5), int(image.shape[0]*0.5)))
        # image = image[360:,:]
        self.input = image

    def get_output(self):
        return self.output

    def sift_locate(self):
        # Temporarily only support one template
        self.input_kp, self.input_desc = self.sift.detectAndCompute(self.input, None)
        matches = self.flann.knnMatch(self.tmplt_desc[0], self.input_desc,k=2)

        good_points = []
        # Ratio test. Lower better
        for m,n in matches: #m: query images, n: from grayframe
            if m.distance < 0.6*n.distance:
                good_points.append(m)
        self.output = good_points 

        modified_img = self.input.copy()
        if self.debug_flags["template_kp"] == True:
            self.debug_imgs["template_kp"] = cv2.drawKeypoints(self.templates[0], self.tmplt_kp[0], self.templates[0])
        
        if (self.debug_flags["input_kp"] == True):
            self.debug_imgs["input_kp"] = cv2.drawKeypoints(modified_img, self.input_kp, modified_img)
        if self.debug_flags["homogr"] == True:
            self.debug_imgs["homogr"] = self.draw_homography(self.input_kp, self.tmplt_kp[0], good_points)
        if self.debug_flags["matches"] == True:
            self.debug_imgs["matches"] = cv2.drawMatches(self.templates[0], self.tmplt_kp[0], modified_img, self.input_kp, good_points, modified_img)
        self.input = None


    def draw_homography(self, input_kp, tmplt_kp, good_points):
        modified_img = self.input.copy()


        if len(good_points) >= 1:
            avg_x, avg_y = self.centroid_of_points()
            cv2.circle(modified_img, (avg_x, avg_y), 7, (0,255,0), -1)
            


        if len(good_points) > 3:
            # Extract position of points and reformat array
            query_pts = np.float32([tmplt_kp[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
            train_pts = np.float32([input_kp[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)

            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            # Perspective transform
            s = self.templates[0].shape
            pts = np.float32([[0, 0], [0, s[0]], [s[1], s[0]], [s[1], 0]]).reshape(-1, 1, 2)
            if (matrix is not None):
                dst = cv2.perspectiveTransform(pts, matrix)
                modified_img = cv2.polylines(modified_img, [np.int32(dst)], True, (255, 0, 0), 3)
        return modified_img

    def centroid_of_points(self):
        if len(self.output) >= 1:
            input_pts = np.float32([self.input_kp[m.trainIdx].pt for m in self.output])
            avg_x = int(sum([pt[0] for pt in input_pts])/len(input_pts))
            avg_y = int(sum([pt[1] for pt in input_pts])/len(input_pts))
            return (avg_x, avg_y)
        else:
            return (0, 0)

    def update_debug_flags(self, flags):
        """Updates the debug flags so that procesor will create debug images if desired.

        Args:
            flags (dict(boolean)): dictionary that maps the flag keys (as defined in __init__ to True or False).
            Not necessary to include all of the flags to update them.
        """
        for f in flags.keys():
            try:
                a = self.debug_flags[f] # Don't permit creation of new flags
                self.debug_flags[f] = flags[f] 
            except KeyError:
                pass

    def show_debug(self, flag):
        if (self.debug_imgs[flag] is not None):
            cv2.imshow(flag, self.debug_imgs[flag])
            cv2.waitKey(1)

