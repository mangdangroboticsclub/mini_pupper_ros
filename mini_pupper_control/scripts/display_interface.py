#!/usr/bin/python3

import PIL
import rospy
import sensor_msgs
import cv2
from cv_bridge import CvBridge
from MangDang.LCD.ST7789 import ST7789

def callback(data):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    image = PIL.Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
    resized = image.resize((320, 240))
    disp.display(resized)

def listener():
    rospy.init_node('display_interface', anonymous=True)
    rospy.Subscriber('mini_pupper_lcd/image_raw',
                     sensor_msgs.msg.Image, callback)
    rospy.spin()

if __name__ == '__main__':
    disp = ST7789()
    disp.begin()
    disp.clear()
    listener()
