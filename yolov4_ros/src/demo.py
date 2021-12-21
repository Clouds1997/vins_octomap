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

"""hyper parameters"""
use_cuda = True

cfgfile = '../models/yolov4.cfg'
weightfile = '../models/yolov4.weights'
m = Darknet(cfgfile)

m.print_network()
m.load_weights(weightfile)
print('Loading weights from %s... Done!' % (weightfile))

if use_cuda:
    m.cuda()

num_classes = m.num_classes
if num_classes == 20:
    namesfile = 'data/voc.names'
elif num_classes == 80:
    namesfile = 'data/coco.names'
else:
    namesfile = 'data/x.names'
class_names = load_class_names(namesfile)

def callback(data):
    bridge = CvBridge()
    try:
      img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    sized = cv2.resize(img, (m.width, m.height))
    sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

    boxes = do_detect(m, sized, 0.5, 0.4, use_cuda)

    img_plot = plot_boxes_cv2(img, boxes[0], class_names=class_names)

    cv2.imshow("test", img_plot)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/d400/color/image_raw", Image, callback)
    rospy.spin()
