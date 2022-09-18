#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
import math

pose = Pose()

def callback(data):
    global pose
    d = 0.25
    pub_pose = rospy.Publisher('body_pose', Pose, queue_size=10)
    rate = rospy.Rate(200)
    if(data.orientation.w!=pose.orientation.w):
        pose.orientation.w = pose.orientation.w + d*abs(data.orientation.w-pose.orientation.w)*(data.orientation.w-pose.orientation.w)/abs(data.orientation.w-pose.orientation.w)
    if(data.orientation.x!=pose.orientation.x):
        pose.orientation.x = pose.orientation.x + d*abs(data.orientation.x-pose.orientation.x)*(data.orientation.x-pose.orientation.x)/abs(data.orientation.x-pose.orientation.x)
    if(data.orientation.y!=pose.orientation.y):
        pose.orientation.y = pose.orientation.y + d*abs(data.orientation.y-pose.orientation.y)*(data.orientation.y-pose.orientation.y)/abs(data.orientation.y-pose.orientation.y)
    if(data.orientation.z!=pose.orientation.z):
        pose.orientation.z = pose.orientation.z + d*abs(data.orientation.z-pose.orientation.z)*(data.orientation.z-pose.orientation.z)/abs(data.orientation.z-pose.orientation.z)
    pose.position.x=data.position.x
    pose.position.y=data.position.y
    pose.position.z=data.position.z
    pub_pose.publish(pose)
    #rospy.loginfo(pose)

def listener():
    rospy.init_node('pose_controller', anonymous=True)
    rospy.Subscriber("target_body_pose", Pose, callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
