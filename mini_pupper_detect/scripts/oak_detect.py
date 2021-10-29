#!/usr/bin/env python3
import rospy
#from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection2DArray
import math

height = 300
width = 300
roll=0
pitch=0
yaw = 0
yaw_increment=0
pitch_increment=0
pose = Pose()

#mobilenet object list
#0: background
#1: aeroplane
#2: bicycle
#3: bird
#4: boat
#5: bottle
#6: bus
#7: car
#8: cat
#9: chair
#10: cow
#11: diningtable
#12: dog
#13: horse
#14: motorbike
#15: person
#16: pottedplant
#17: sheep
#18: sofa
#19: train
#20: tvmonitor

def toward_obj(obj_class,obj_list):
    global pose,roll,pitch,yaw,yaw_increment,pitch_increment
    rate = rospy.Rate(200) # 200hz
    yaw_increment = 0
    pitch_increment = 0
    for i in obj_list:
        bb = i.bbox #BoundingBoxes
        si = i.source_img #SourceImage
        r = i.results #Results
        rr = r[0] #RealResults
        #print(rr.id)
        if(rr.id == obj_class):
            print(11111)
            yaw_increment=(width/2-bb.center.x)*0.0002
            pitch_increment=-(height/2-bb.center.y)*0.0002
            
        #else:
            #yaw_increment=0
    yaw = yaw+yaw_increment
    #print(yaw_increment)
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

    pub_pose.publish(pose)
    rate.sleep()

def callback(data):
    bounding_boxes = data
    detections = bounding_boxes.detections
    toward_obj(5,detections)
    #toward_obj('cup',a)
    #print(a[0])
    
def listener():
    rospy.init_node('yolo_detect', anonymous=True)
    rospy.Subscriber("/mobilenet_publisher/color/mobilenet_detections", Detection2DArray, callback)
    rospy.spin()
 
if __name__ == '__main__':
    pub_pose = rospy.Publisher('mini_pupper/body_pose', Pose, queue_size=10)
    listener()
