# mini_pupper_gazebo

You can also play with Mini Pupper with only your laptop or PC.  
**Raspberry Pi does not have enough resources to simulate.**
**Notice: Setting the param `/use_sim_time` as `true` is required for simulation.**

![nav](../imgs/instruction.gif)

Launch the Gazebo simulator with the following command.  
The param `/use_sim_time` is set to `true` by this launch file.

```sh
roslaunch mini_pupper_gazebo gazebo.launch
```

Run the following command in another terminal to send command to the robot.

```sh
roslaunch champ_teleop teleop.launch
```

