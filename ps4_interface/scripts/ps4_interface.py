#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

vel = Twist()
pose = Pose()
MAX_X = rospy.get_param('/gait/max_linear_velocity_x')
MAX_Y = rospy.get_param('/gait/max_linear_velocity_y')
MAX_Z = rospy.get_param('/gait/max_angular_velocity_z')
HEIGHT = rospy.get_param('/gait/nominal_height')
pose.position.z=0
roll = 0
roll_max = 0.3

def callback(data):
    global vel, pose, roll
    axes = data.axes
    buttons = data.buttons

    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_pose = rospy.Publisher('target_body_pose', Pose, queue_size=10)
    rate = rospy.Rate(200) 


    if not buttons[7]:
        #cmd_vel control
        vel.linear.x = axes[1]*MAX_X
        vel.linear.y = axes[0]*MAX_Y
        vel.angular.z=2*axes[2]*MAX_Z
        pitch = 0
        yaw = 0

    else:
        #pose(pitch&yaw) control
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        pitch = -axes[5]*0.4
        yaw = axes[2]*0.5

    #height control
    if(axes[10]==-1):
        pose.position.z = pose.position.z - 0.0005
    elif(axes[10]==1):
        pose.position.z = pose.position.z + 0.0005
    if(pose.position.z>=0):
        pose.position.z=0
    elif(pose.position.z<=-1*HEIGHT+0.03):
        pose.position.z=-1*HEIGHT+0.03
    
    #roll control
    if(axes[9]==1):
        roll = roll - 0.005
    elif(axes[9]==-1):
        roll = roll + 0.005
    if(roll>=roll_max):
        roll=roll_max
    elif(roll<=-1*roll_max):
        roll=-1*roll_max
    
    #clear button
    if buttons[6]:
        roll = 0
        pitch = 0
        yaw = 0
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        pose.position.z=0
        
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
        
    pub_vel.publish(vel)
    pub_pose.publish(pose)
    #rospy.loginfo(pose)

def listener():
    rospy.init_node('ps4_interface', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
