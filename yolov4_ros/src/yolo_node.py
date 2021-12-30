#!/usr/bin/python3
import sys
import os
import platform
import getpass
from unicodedata import name
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import yolo_detect
from image_obj_msgs.msg import ImageObj
from image_obj_msgs.msg import ImageBox



class App:
    def __init__(self):
        rospy.init_node('yolo_detector', log_level=rospy.INFO)
        self.bridge = CvBridge()
        self.detector = None
        # self.enable_rate_info = _enable_rate_info
        # self.enable_floating_window = _enable_floating_window
        self.pre_time = rospy.Time.now()
        self.pre_rate = 0.0

    def _init_params(self):
        self.sub_image_topic = rospy.get_param("~input_image_topic", default="/d400/color/image_raw")
        self.cfgfile = rospy.get_param("~model_file", default="/home/autoware/shared_dir/workspace/yolo_ws/src/yolov4_ros/models/yolov4.cfg")
        self.weightfile = rospy.get_param("~prototxt_file", default="/home/autoware/shared_dir/workspace/yolo_ws/src/yolov4_ros/models/yolov4.weights")
        self.namesfile = rospy.get_param("~namesfile", default="/home/autoware/shared_dir/workspace/yolo_ws/src/yolov4_ros/data/coco.names")
        self.use_cuda = rospy.get_param("~use_cuda", default=True)

        self.detector = yolo_detect.Detector(self.cfgfile,self.weightfile,self.namesfile,self.use_cuda)

    def _add_pub(self):
        self.pub_image_detected = rospy.Publisher("/image/detected", Image, queue_size=1)
        self.pub_image_obj = rospy.Publisher("/image/detected_obj", ImageObj, queue_size=1)

    def _add_sub(self):
        rospy.Subscriber(self.sub_image_topic, Image, self._cb_image, queue_size=1,buff_size=2**24)

    def run(self):
        self._init_params()
        self._add_pub()
        self._add_sub()
        rospy.spin()

    def _cb_image(self, image):
        # rospy.loginfo("[image-light-detector] input image size: [{}, {}]".format(image.width, image.height))
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logwarn("Cannot convert ros image to cv image, err: {}".format(e))
            return
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv_image,boxes = self.detector.realtime_detect(cv_image)
        # if self.enable_rate_info:
        #     time_now = rospy.Time.now()
        #     time_interval = (time_now - self.pre_time).to_sec()
        #     if time_interval > 0:
        #         rate = 1.0 / time_interval
        #         rate = (rate + self.pre_rate) / 2.0
        #     else:
        #         rate = 0.0
        #     self.pre_rate = rate
        #     cv2.putText(cv_image, "FPS {:.2f}".format(rate), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        # if self.enable_floating_window:
        #     cv2.imshow('image-detected', cv_image)
        
        ros_image = self.bridge.cv2_to_imgmsg(cv_image)
        ros_image.header.frame_id = image.header.frame_id
        ros_image.header.stamp = image.header.stamp
        # rospy.loginfo("[image-light-detector] out image size: [{}, {}]".format(ros_image.width, ros_image.height))
        self.pub_image_detected.publish(ros_image)
        self.pre_time = rospy.Time.now()

        boxes_msg = ImageObj()
        boxes_msg.header = image.header

        boxes_tmp = []
        # print(boxes)

        if(len(boxes[0])):
            for i in range(len(boxes[0])):
                box_msg = ImageBox()
                box_msg.box = boxes[0][i]
                boxes_tmp.append(box_msg)

        boxes_msg.boxes = boxes_tmp

        self.pub_image_obj.publish(boxes_msg)


if __name__ == "__main__":
    # enable_floating_window = rospy.get_param("~enable_floating_window", default=True)
    # enable_rate_info = rospy.get_param("~enable_rate_info", default=True)
    app = App()
    app.run()
    