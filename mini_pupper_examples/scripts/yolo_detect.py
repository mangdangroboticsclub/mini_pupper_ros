#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

height = 480
width = 640
roll=0
pitch=0
yaw = 0
yaw_increment=0
pitch_increment=0
pose = Pose()
vel = Twist()

def toward_obj(obj_class,obj_list):
    global pose,roll,pitch,yaw,yaw_increment,pitch_increment
    rate = rospy.Rate(200) # 10hz

    for i in obj_list:
        if(i.Class==obj_class):
            yaw_increment=(width/2-(i.xmin+i.xmax)/2)*0.0008
            
            
            pitch_increment=-(height/2-(i.ymin+i.ymax)/2)*0.00008
            
        #else:
            #yaw_increment=0
    yaw = 0#yaw+yaw_increment
    print(yaw_increment)
    pitch = pitch+pitch_increment
    cy=math.cos(yaw*0.5)
    sy=math.sin(yaw*0.5)
    cp=math.cos(pitch*0.5)
    sp=math.sin(pitch*0.5)
    cr =math.cos(roll * 0.5)
    sr =math.sin(roll * 0.5)

    pose.orientation.w= cy * cp * cr + sy * sp * sr
    pose.orientation.x = cy * cp * sr - sy * sp * cr
    pose.orientation.y = sy * cp * sr + cy * sp * cr
    pose.orientation.z = sy * cp * cr - cy * sp * sr

    vel.linear.y = yaw_increment
    pub_pose.publish(pose)
    pub_vel.publish(vel)
    rate.sleep()

def callback(data):
    a = data.bounding_boxes
    toward_obj('bottle',a)
    toward_obj('cup',a)
    #print(a[0])
    
def listener():
    rospy.init_node('yolo_detect', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
    rospy.spin()
 
if __name__ == '__main__':
    pub_pose = rospy.Publisher('body_pose', Pose, queue_size=10)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    listener()
