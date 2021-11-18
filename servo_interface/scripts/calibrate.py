#!/usr/bin/python3
import os
import rospy
import yaml
import io

servo_pins = [15,14,13,   12,11,10,   9,8,7,   6,5,4] #rf lf rb lb
key_list = [\
'rf1_initial_angle','rf2_initial_angle','rf3_initial_angle',\
'lf1_initial_angle','lf2_initial_angle','lf3_initial_angle',\
'rb1_initial_angle','rb2_initial_angle','rb3_initial_angle',\
'lb1_initial_angle','lb2_initial_angle','lb3_initial_angle']
calibration_dict = {}

#initialize(used by developers)
#for i in servo_pins:
#    p = "/sys/class/pwm/pwmchip0/pwm"+str(i)+"/duty_cycle"
#    os.system('sudo chmod 777 '+p)
#    print(p)

def set_servo_angle(pin,angle):
    duty_cycle = int(500000+11111.11111*angle)
    servo_pin = "/sys/class/pwm/pwmchip0/pwm"+str(pin)+"/duty_cycle"
    f = open(servo_pin, "w")
    f.write(str(int(duty_cycle)))

if __name__=='__main__':
    rospy.init_node('servo_calibration',anonymous=True)
    calibrated_angles = []
    flag = ""
    i = 0
    for pin in servo_pins:
        print("you are calibrating is servo "+str(i+1))
        while(1):
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
        calibrated_angles.append(float(angle))
        current_key = key_list[i]
        calibration_dict[current_key] = angle
        i = i + 1
        print(calibration_dict)
    print("calibration done!")
    current_path = os.path.dirname(os.path.abspath(__file__))
    current_path = os.path.join(current_path,os.path.pardir)
    calibration_yaml_path = os.path.join(current_path,'config/calibration/calibration.yaml')
    with open(calibration_yaml_path, 'w') as f:
        yaml.dump(calibration_dict,f,default_flow_style=False)
