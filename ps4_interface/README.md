# ps4_ros
Sony PlayStation 4 DualShock®4 node joy_msg to twist_msg

# Installation
1. Install __ds4dr__: `$sudo pip install ds4drv`
1. Install __ros-joy__: http://wiki.ros.org/joy
1. Go into pairing mode with PS4: Playstation button + share button for ~5 sec
1. Run `$ds4drv` from command line to connect to PS4
  1. This will output something like _Created devices /dev/input/jsX
  1. remember /dev/input/js__X__ and update the launch file (default X=0)
  1. `$sudo chmod a+rw /dev/input/jsX`

# Starting
## One by one
1. `$ds4drv` if it is not running already.
1. `$rosparam set joy_node/dev "/dev/input/jsX"`
1. `$rosrun joy joy_node`
1. `$rosrun ps4_ros ps4_ros`

## roslaunch
1. `$roslaunch ps4_ros ps4`


* One can adjust the following parameter inside the launch file or use rosparam by using the _one by one_ start procedure

  * ``<param name="scale_angular" value="1.5"/>``

  * ``<param name="scale_linear" value="0.5"/>``

# Troubleshooting

* Display raw _ds4drv_ data
  * `$sudo jstest /dev/input/jsX`
  
# To Do (v.2.0)
* [ ] Pan-and-Tilt on second joystick
