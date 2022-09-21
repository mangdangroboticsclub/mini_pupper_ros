#!/usr/bin/python3
import os
import rospy
import yaml
import io
import re
import numpy as np

servo_pins = [15,14,13,   12,11,10,   9,8,7,   6,5,4] #rf lf rb lb

def set_servo_angle(pin,angle):
    duty_cycle = int(500000+11111.11111*angle)
    servo_pin = "/sys/class/pwm/pwmchip0/pwm"+str(pin)+"/duty_cycle"
    with open(servo_pin, "w") as f:
        f.write(str(int(duty_cycle)))

if __name__=='__main__':
    rospy.init_node('servo_calibration',anonymous=True)
    calibrated_angles = []
    flag = ""
    i = 0
    for pin in servo_pins:
        print("you are calibrating servo "+str(i+1))
        while not(rospy.is_shutdown()):
            angle = input("input an angle: ")
            set_servo_angle(pin,float(angle))
            flag = input("is this angle ok?(y/n)")
            print(flag)
            if(flag == "y" or flag == "Y" or flag == "yes"):
                break
            elif(flag == "n" or flag == "N" or flag == "no"):
                pass
            else:
                print("wrong input, please do it again")
        calibrated_angles.append(int(angle))
        i = i + 1
        print(calibrated_angles)
    print("calibration done!")
    calibration_path = '/sys/bus/nvmem/devices/3-00500/nvmem'
    #open(calibration_path, 'w').close()
    #with open(calibration_path, 'w') as eeprom:
    #    eeprom.truncate(0)
    #    for i in calibrated_angles:
    #        eeprom.write(i)

    buf_matrix = np.zeros((3,4))
    buf_matrix[0][0] = 90-calibrated_angles[0]
    buf_matrix[0][1] = 90-calibrated_angles[3]
    buf_matrix[0][2] = calibrated_angles[6]-90
    buf_matrix[0][3] = calibrated_angles[9]-90

    buf_matrix[1][0] = calibrated_angles[1]-45
    buf_matrix[1][1] = 135-calibrated_angles[4]
    buf_matrix[1][2] = calibrated_angles[7]-45
    buf_matrix[1][3] = 135-calibrated_angles[10]

    buf_matrix[2][0] = calibrated_angles[2]-135
    buf_matrix[2][1] = 45-calibrated_angles[5]
    buf_matrix[2][2] = calibrated_angles[8]-90-135
    buf_matrix[2][3] = 45-calibrated_angles[11]

    p1 = re.compile("([0-9]\.) ( *)")
    partially_formatted_matrix = p1.sub(r"\1,\2", str(buf_matrix))
    p2 = re.compile("(\]\n)")
    matrix = p2.sub("],\n", partially_formatted_matrix)

    with open(calibration_path, "w") as nv_f:
        _tmp = str(buf_matrix)
        _tmp = _tmp.replace('.' , ',')
        _tmp = _tmp.replace('[' , '')
        _tmp = _tmp.replace(']' , '')
        print(_tmp, file = nv_f)
        nv_f.close()
