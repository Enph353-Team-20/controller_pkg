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
ID_NET = '/id_net'

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
        # self.id_net = NeuralNet(ID_NET)
        self.sub = rospy.Subscriber("/plate_imgs",Image,self.callback)
        self.pub = rospy.Publisher("/license_plate", String)
        self.bridge = CvBridge()

    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        cv_img = cv_img / 255

        letter_imgs = np.array([
            cv_img[:,0:120],
            cv_img[:,100:220],
            cv_img[:,280:400],
            cv_img[:,380:500],
            cv_img[:,500:650]
        ])

        # cv2.imshow("huh", letter_imgs[3])
        # cv2.waitKey(1)
        
        one_hots = self.plate_net.predictImages(letter_imgs)
        # print("Hmm: ")
        # print(one_hots)

        plate_prediction = ""
        for oh in one_hots[0:2]:
            plate_prediction += onehot_to_sym(oh, "letters")
        for oh in one_hots[2:4]:
            plate_prediction += onehot_to_sym(oh, "nums")
        id_prediction = onehot_to_sym(one_hots[4], "nums")

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



