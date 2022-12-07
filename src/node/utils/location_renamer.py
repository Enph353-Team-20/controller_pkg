#!/usr/bin/env python3

import os
import cv2
from time import time

inpath = "/home/fizzer/Downloads/CarIDData"
outpath = "/home/fizzer/Downloads/renamed_ids"

filenames = os.listdir(inpath)

for fn in filenames:
    img = cv2.imread(inpath + '/' + fn)
    cv2.imshow('Plate', img)
    cv2.waitKey(1)
    correct_id = input("Enter ID:")
    new_name = outpath + '/' + "P" + correct_id + "AA00_" + str(int(time())) + ".png"
    os.rename(inpath + '/' + fn, new_name)

