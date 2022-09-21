#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
import math

pose = Pose()
target = Pose()

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radius

def quaternion_from_euler(roll, pitch, yaw):
    cy=math.cos(yaw*0.5)
    sy=math.sin(yaw*0.5)
    cp=math.cos(pitch*0.5)
    sp=math.sin(pitch*0.5)
    cr =math.cos(roll * 0.5)
    sr =math.sin(roll * 0.5)

    w= cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr  
    
    return x, y, z, w
    
def callback(data):
    global target
    target = data


if __name__ == '__main__':
    rospy.init_node('pose_controller', anonymous=True)
    rospy.Subscriber("target_body_pose", Pose, callback)
    pub_pose = rospy.Publisher('body_pose', Pose, queue_size=10)
    rate = rospy.Rate(200)
    d = 0.0001
    while not rospy.is_shutdown():
        pose_r,pose_p,pose_y = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        target_r,target_p,target_y = euler_from_quaternion(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w)
        if(target_r<pose_r):
            pose_r = pose_r - d
        elif(target_r>pose_r):
            pose_r = pose_r + d
        if(target_p<pose_p):
            pose_p = pose_p - d
        elif(target_p>pose_p):
            pose_p = pose_p + d
        if(target_y<pose_y):
            pose_y = pose_y - d
        elif(target_y>pose_y):
            pose_y = pose_y + d
        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = quaternion_from_euler(pose_r, pose_p, pose_y)
        pose.position.x=target.position.x
        pose.position.y=target.position.y
        pose.position.z=target.position.z
        pub_pose.publish(pose)
        #rospy.loginfo(pose)
