#!/usr/bin/env python3

import numpy as np
import os
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2


from collections import Counter
from matplotlib import pyplot as plt

from tensorflow.keras import models

from std_msgs.msg import String

PLATE_NET = '/home/fizzer/ros_ws/src/controller_pkg/src/node/plate_net'
ID_NET = '/home/fizzer/ros_ws/src/controller_pkg/src/node/id_net'

class NeuralNet():
    def __init__(self, model_path):
        self.__LEARNING_RATE__ = 1e-4
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
        # os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"
        self.conv_model = models.load_model(model_path)
        pass

    def predictImages(self, imgs):
        """Send an array of images for the network to predict.

        Args:
            imgs (np.array(np.array())): A np array containing images, which are themselves np arrays.

        Returns:
            _type_: One_hot vector
        """
        predicted = self.conv_model.predict(imgs)
        return predicted
    
class NeuralNetManager():
    def __init__(self):
        self.plate_net = NeuralNet(PLATE_NET + '/e353_plate_model.h5')
        self.id_net = NeuralNet(ID_NET + '/e353_id_model.h5')
        self.sub = rospy.Subscriber("/plate_imgs",Image,self.callback)
        self.pub = rospy.Publisher("/license_plate", String,queue_size=1)
        self.bridge = CvBridge()



    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        cv_img = cv_img / 255

        id_num = cv_img[:,500:650,:]
        hsv_num = cv2.cvtColor(np.float32(id_num), cv2.COLOR_BGR2HSV)

        rows = id_num.shape[0]
        cols = id_num.shape[1]

        lower = np.array([0,0,0])
        upper = np.array([0,0,40])
        mask = cv2.inRange(hsv_num,lower,upper)

        for i in range(rows):
            for j in range(cols):
                if mask[i,j] > 0:
                    id_num[i,j,2] = 120/255
                else:
                    id_num[i,j,:] = 1
        
        id_img = np.array([id_num])

        letter_imgs = np.array([
            cv_img[:,0:120],
            cv_img[:,100:220],
            cv_img[:,280:400],
            cv_img[:,380:500]
        ])
        # get plate id image and convert to binary
        # id_gray = np.float32(cv2.cvtColor(np.float32(cv_img[:,500:650,:]), cv2.COLOR_BGR2GRAY))
        # ret, id_bin = cv2.threshold(id_gray, 25, 255, cv2.THRESH_BINARY)
        # id_img = np.array([id_bin])

        # cv2.imshow("huh", letter_imgs[3])
        # cv2.waitKey(1)
        
        plate_one_hots = self.plate_net.predictImages(letter_imgs)
        # id_one_hot = self.id_net.predictImages(id_img)
        id_one_hot = self.plate_net.predictImages(id_img)

        plate_prediction = ""
        for oh in plate_one_hots[0:2]:
            plate_prediction += onehot_to_sym(oh, "letters")
        for oh in plate_one_hots[2:4]:
            plate_prediction += onehot_to_sym(oh, "nums")
        # id_prediction = onehot_to_sym(id_one_hot[0], "nums")
        id_prediction = onehot_to_sym(id_one_hot[0], "nums")

        self.pub.publish(str('Team20,silvertip,' + str(id_prediction) + ',' + plate_prediction))
    

def remap_sym(sym):
    if sym >= '0' and sym <= '9':
        return ord(sym) - ord('0')
    else:
        return ord(sym) - ord('A') + 10

def onehot_to_sym(one_hot, focus="all"):
    prob = 0
    if focus == "letters":
        rel_range = (10, len(one_hot))
        idx = 10
    elif focus == "nums":
        rel_range = (0, 10)
        idx = 0
    else:
        rel_range = (0, len(one_hot))
        idx = 0

    for i in range(rel_range[0], rel_range[1]):
        if one_hot[i] > prob:
            idx = i
            prob = one_hot[i]
    
    if idx <= 9:
        return chr(idx+ord('0'))
    else:
        return chr(idx-10+ord('A'))



