# mini_pupper_gazebo

You can also play with Mini Pupper with only your laptop or PC.  
**Raspberry Pi does not have enough resources to simulate.**

![nav](../imgs/instruction.gif)

Launch the Gazebo simulator with the following command.

```sh
roslaunch mini_pupper_gazebo gazebo.launch
```

Run the following command in another terminal to send command to the robot.

```sh
roslaunch champ_teleop teleop.launch
```
