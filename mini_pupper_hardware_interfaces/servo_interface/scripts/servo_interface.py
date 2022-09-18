#!/usr/bin/python3
import os
import sys
import time
import rospy
from trajectory_msgs.msg import JointTrajectory
import numpy as np

servo_pins = [15,14,13,  12,11,10,  9,8,7,  6,5,4] #rf lf rb lb
pi_value =3.1415926535
servo_configs = []

def set_servo_angle(pin,angle):
	duty_cycle = int(500000+11111.11111*angle)
	servo_pin = "/sys/class/pwm/pwmchip0/pwm"+str(pin)+"/duty_cycle"
	f = open(servo_pin, "w")
	f.write(str(int(duty_cycle)))

def get_param():
    global servo_configs
    eeprom = open("/sys/bus/i2c/devices/3-0050/eeprom",'r',encoding="ISO-8859-1")
    try:
        with open("/sys/bus/i2c/devices/3-0050/eeprom", "rb") as nv_f:
            arr1 = np.array(eval(nv_f.readline()))
            arr2 = np.array(eval(nv_f.readline()))
            matrix = np.append(arr1, arr2)
            arr3 = np.array(eval(nv_f.readline()))
            matrix = np.append(matrix, arr3)
            matrix.resize(3,4)
            rospy.loginfo("Get nv calibration params: "+str(matrix))
    except:
        matrix = np.array([[0, 0, 0, 0], [45, 45, 45, 45], [-45, -45, -45, -45]])
        rospy.loginfo("Get nv calibration paramsfailed, set to default params: "+str(matrix))
    line1 = matrix[0].tolist()
    line2 = matrix[1].tolist()
    line3 = matrix[2].tolist()
    servo_configs.append(90-line1[0])
    servo_configs.append(45+line2[0]+45)
    servo_configs.append(135+line3[0]+45)

    servo_configs.append(90-line1[1])
    servo_configs.append(135-line2[1]-45)
    servo_configs.append(45-line3[1]-45)

    servo_configs.append(90+line1[2])
    servo_configs.append(45+line2[2]+45)
    servo_configs.append(135+line3[2]+45)

    servo_configs.append(90+line1[3])
    servo_configs.append(135-line2[3]-45)
    servo_configs.append(45-line3[3]-45)

def callback(data):
    global servo_pins
    points = data.points[0]
    joint_positions = points.positions
    lf1_position = (180/pi_value)*joint_positions[0]
    lf2_position = (180/pi_value)*joint_positions[1]
    lf3_position = (180/pi_value)*joint_positions[2]
    rf1_position = (180/pi_value)*joint_positions[3]
    rf2_position = (180/pi_value)*joint_positions[4]
    rf3_position = (180/pi_value)*joint_positions[5]
    lb1_position = (180/pi_value)*joint_positions[6]
    lb2_position = (180/pi_value)*joint_positions[7]
    lb3_position = (180/pi_value)*joint_positions[8]
    rb1_position = (180/pi_value)*joint_positions[9]
    rb2_position = (180/pi_value)*joint_positions[10]
    rb3_position = (180/pi_value)*joint_positions[11]

    a1 = servo_configs[0]+rf1_position
    a2 = servo_configs[1]-rf2_position
    a3 = servo_configs[2]-90-rf2_position-rf3_position

    a4 = servo_configs[3]+lf1_position
    a5 = servo_configs[4]+lf2_position
    a6 = servo_configs[5]+90+lf2_position+lf3_position

    a7 = servo_configs[6]-rb1_position
    a8 = servo_configs[7]-rb2_position
    a9 = servo_configs[8]-90-rb2_position-rb3_position

    a10 = servo_configs[9]-lb1_position
    a11 = servo_configs[10]+lb2_position
    a12 = servo_configs[11]+90+lb2_position+lb3_position
    
    set_servo_angle(15,a1)
    set_servo_angle(14,a2)
    set_servo_angle(13,a3)

    set_servo_angle(12,a4)
    set_servo_angle(11,a5)
    set_servo_angle(10,a6)

    set_servo_angle(9,a7)
    set_servo_angle(8,a8)
    set_servo_angle(7,a9)

    set_servo_angle(6,a10)
    set_servo_angle(5,a11)
    set_servo_angle(4,a12)

def listener():
    rospy.init_node('servo_interface',anonymous=True)
    get_param()
    rospy.Subscriber("/joint_group_position_controller/command",JointTrajectory
,callback,queue_size=1)
    rospy.spin()

if __name__=='__main__':
    listener()
