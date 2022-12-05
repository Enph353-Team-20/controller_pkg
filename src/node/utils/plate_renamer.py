#!/usr/bin/env python3

import os
import cv2
from time import time

inpath = "/home/fizzer/Downloads/img_spam"
outpath = "/home/fizzer/Downloads/renamed_plates"

filenames = os.listdir(inpath)

for fn in filenames:
    img = cv2.imread(inpath + '/' + fn)
    cv2.imshow('Plate', img)
    cv2.waitKey(1)
    correct_plate = input("Enter AA##: ")
    new_name = inpath + '/' + "P_" + correct_plate + "_" + str(int(time())) + ".png"
    os.rename(outpath + '/' + fn, new_name)

