# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :
'''

# import sys
# import time
# from PIL import Image, ImageDraw
# from models.tiny_yolo import TinyYoloNet
from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet
import argparse

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class Detector:
    def __init__(self, cfgfile, weightfile, namesfile, use_cuda):
        # cfgfile = 'yolo/yolov4.cfg'
        # weightfile = 'yolo/yolov4.weights'
        self.model = Darknet(cfgfile)
        self.model.load_weights(weightfile)
        if(use_cuda):
            print("cuda ***************************************************")
            self.model.cuda()
        self.bridge = CvBridge()
        self.use_cuda = use_cuda
        self.class_names = load_class_names( namesfile)


    
    def realtime_detect(self, image):

        sized = cv2.resize(image, (self.model .width,  self.model .height))
        # sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)
        width = image.shape[1]
        height = image.shape[0]
        boxes = do_detect(self.model, sized, 0.5, 0.4, self.use_cuda)
        img = image
        boxsrc = []
        for box in boxes[0]:
            x1 =max( int(box[0] * width),0)
            y1 = max(int(box[1] * height),0)
            x2 = min(int(box[2] * width),width-1)
            y2 = min(int(box[3] * height),height-1)
            left = x1 - y1/2
            top = y1 - y2 /2
            # img = cv2.rectangle(img, (x1,y1), (x2, y2), (255, 0, 0), 1)
            boxsrc.append([x1,y1,x2,y2,box[4],box[5],box[6]])
        # cv2.imshow('rectangle', img)
        # cv2.waitKey(1)

        # img_plot = plot_boxes_cv2(image, boxes[0], class_names=self.class_names)
        # cv2.imshow('rectangle', img_plot)
        # cv2.waitKey(1)
        return image,[boxsrc]
