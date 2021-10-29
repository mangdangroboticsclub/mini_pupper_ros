#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import Image as Image_ROS
from cv_bridge import CvBridge, CvBridgeError
import cv2

import os
import sys
import time
from PIL import Image

sys.path.append("/home/ubuntu/MiniPupperROS/mangdang")
sys.path.extend([os.path.join(root, name) for root, dirs, _ in os.walk("/home/ubuntu/MiniPupperROS/mangdang") for name in dirs])
from LCD.ST7789 import ST7789

def callback(data):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(data,"bgr8")
    image = Image.fromarray(cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB))
    image.resize((320,240))
    disp.display(image)

def showImage():
    rospy.init_node('display_interface',anonymous = True)
    rospy.Subscriber('usb_cam/image_raw', Image_ROS, callback)
    rospy.spin()

if __name__ == '__main__':
    disp = ST7789()
    disp.begin()
    disp.clear()
    showImage()
