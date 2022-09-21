# mini_pupper_examples


## 1.Computer Vision

We've tested the integration with OAK-D-Lite. It's a great platform to run some deep learning models.  
You can follow these commands to try this demo(make sure you have connected OAK-D-Lite to Mini Pupper).

```sh
roslaunch mini_pupper_examples follow_object.launch
```

![Mini Pupper OpenCV Object Detection](../imgs/OpenCV.ObjDetect.gif)

Mini Pupper OpenCV Object Detection Demo

Also, if you want to do some CV projects, you can add a usb camera on Mini Pupper, and subscribe the comressed image on your PC.  
The transportation of raw image through network will be too slow, you may need to use image_transport to turn the compressed image to normal image and then use it.

```sh
rosrun image_transport republish compressed in:=(in_base_topic) raw out:=(out_base_topic)
```

## 2. Dancing motion

TBD
