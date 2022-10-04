## MiniPupper Teleop

![Build Status](https://github.com/sskorol/minipupper-teleop/actions/workflows/main.yml/badge.svg?branch=main)

This project allows streaming video from [MiniPupper](https://minipupperdocs.readthedocs.io/en/latest/) via WebRTC and teleoperating it via ROS. Note that the [backend](https://github.com/sskorol/minipupper-teleop/tree/main/backend) expects you've already connected [OAK-D Lite](https://shop.luxonis.com/products/oak-d-lite-1) camera to your robot. If you don't have one yet, you can still teleoperate the robot via keyboard, but w/o a camera stream. Also, note that technically you are not forced to use this project with MiniPupper only. It should work for any robot with OAK camera.

<video src='https://user-images.githubusercontent.com/6638780/184704954-94f721f5-6219-48b2-9696-3da01e509eec.mp4'></video>

- [MiniPupper Teleop](#minipupper-teleop)
  - [Architecture](#architecture)
  - [Installation](#installation)
  - [Running on MiniPupper](#running-on-minipupper)
  - [Simulated Environment](#running-in-simulated-environment)
  - [Building FE and BE](#building-fe-and-be)
  - [Known Issues](#known-issues)
  - [ToDo](#todo)

### Architecture

The following diagram reflects the most recent implementation:

![RoboticsDiagram](https://user-images.githubusercontent.com/6638780/184701436-863ccb48-9bcd-44c4-ba8f-c6ebf35d10d9.png)

- **roscore**: a master node that handles all the ROS requests;
- **rosbridge**: WS proxy between FE and ROS that accepts messages (keys) from the remote browser and passes them to ROS nodes;
- **webrtc-be**: Python BE, which streams OAK-D Lite camera video to the remote browser via WebRTC (internally uses [DepthAI](https://docs.luxonis.com/projects/api/en/latest/index.html) API);
- **teleop-fe**: ReactJS FE which uses [roslibjs](https://github.com/RobotWebTools/roslibjs) to communicate with ROS bridge and WebRTC API for camera streaming;
- **servo-drv**: MiniPupper's servo driver node, which listens to CHAMP messages and changes joints' angles;
- **CHAMP/vel-smoother**: [CHAMP](https://github.com/chvmp/champ) framework controls the robot's movements;
- **teleop**: a slightly modified [teleop-legged-robots](https://github.com/SoftServeSAG/teleop_legged_robots) node that accepts remote keys rather than local keyboard events.

Note that `velocity-smoother` was intentionally splitted from `champ` due to netwroking issue mentioned in a [known issues](#known-issues) section.

### Installation

The following [ROS image](https://drive.google.com/file/d/1Mk_bSmIvnN8EIzB8IilS9M4pofTUH9r2/view?usp=sharing) already comes with all the required drivers pre-installed. Just flash it to SD card and you are almost ready to go.

Check the [official guide](https://docs.docker.com/engine/install/ubuntu/) if you don't have Docker yet. Note that you need both `docker` and `docker-compose` CLI tools installed on MiniPupper.

Clone the source code:

```shell
git clone https://github.com/sskorol/minipupper-teleop.git && cd minipupper-teleop
```

Prepare calibration and env files:

```shell
./generate_configs.sh [MINI_PUPPER_IP_ADDRESS]
```

Adjust angles in `calibration_settings.yaml` to match your own robot's calibration data. Note that MiniPupper's legs should be calibrated to 90 degress as on the following screenshot, as CHAMP framework automatically adjusts angles during bringup process to make your robot stand:

![image](https://user-images.githubusercontent.com/6638780/183618832-c133ddef-484c-4974-b6e9-04f7e1d81e6e.png)

MiniPupper's IP is required for the FE container to be able to communicate with the BE via remote browser.

Use the following steps to relax your web-browser restrictions:

- Open Chrome
- Type `chrome://flags/` in the address bar and hit Enter
- Enable `Insecure origins treated as secure` option, and type the IP address of your MiniPupper
- Restart Chrome

### Hardware-related usage tips during local development

#### Battery

The servo interface programmatically adjusts control pins: they are activated on node startup and deactivated on shutdown. On the other hand, MiniPupper's battery service automatically disables servo pins when the power level is low. As you don't want to use a battery while active development (relying on RPi power supply), you should ensure you disable battery service. Otherwise, MiniPupper would fall after bring-up due to automatic pins adjustments.

#### RPi power consumption

When you power MiniPupper's hardware via RPi only, there's a possibility of damaging your main board if you connect lots of add-ons simultaneously. For instance, connecting LiDAR and OAK-D camera + enabling display and servos might bring your RPi into reboot. Disabling a display may do the trick though.

### Running on MiniPupper

Run the following command to start a stack of docker images required to perform teleoperation:

```shell
docker compose pull && docker compose up -d
```

An old docker cli uses a bit different syntax: `docker-compose up -d`.

Open your web browser and go to: `http://[MINI_PUPPER_IP_ADDRESS]`

### Running in simulated environment

If you don't have a robot yet, you can still play with teleoperation locally in a simulated environment.

<video src='https://user-images.githubusercontent.com/6638780/184727365-927b5755-99b4-4098-9010-52444ad33856.mp4'></video>

```shell
# Required for running Gazebo in container
xhost +local:docker
# Download images
docker compose -f docker-compose-sim.yaml pull
# Run services required for simulation
docker compose -f docker-compose-sim.yaml up -d
```

Then open your web-browser on a localhost, wait until teleop is ready, and you're good to go.

### Building FE and BE

Run the following command on MiniPupper to build FE and BE images:

```shell
docker compose build
```

### Local deployment

Create and adjust servo angles for your MiniPupper instance before moving on with the following commands (see `mini_pupper_control_v2` package):
```shell
./create_settings.sh
```

Make sure you started MiniPupper's hardware services:
```shell
roslaunch mini_pupper_bringup bringup.web.launch
```

Install backend dependencies:
```shell
cd backend && ./install.sh
```

Start backend:
```shell
./run.sh
```

Install frontend dependencies:
```shell
cd ../frontend && ./install.sh
```

Prepare `.env` with required environment variables:
```properties
REACT_APP_ROSBRIDGE_SERVER_IP=localhost
REACT_APP_ROSBRIDGE_SERVER_PORT=9090
REACT_APP_RECONNECTION_TIMER=1000
REACT_APP_BE_URL=http://localhost:8080
REACT_APP_IS_SIMULATION=false
```

All the IPs must match the MiniPupper IP if you plan to start Web UI from the remote host. Web browser treats all the addresses relately to the place where it's opened. So if you specify `localhost`, it'll try to find the server on the host machine instead of a real server.

Also note that you don't wan't to enable emulation on MiniPupper as it assumes using Gazebo.

Start frontend:
```shell
npm start
```

Go to [http://MiniPupper:3000](http://MiniPupper:3000) to see the web UI.

Note that frontend and backend depend on the ROS bridge and a custom teleop node. Make sure you've started them beforehand.

### Known issues

In rare cases `teleop`, `smoother` and `servo` nodes can't correctly publish/subscribe to `/cmd_vel` topic due to registration failure. The current workaround is displayed on the following diagram.

You can try to restart docker images to see if it helps. I couldn't yet found the root cause of these Docker <---> ROS networking issues. Feel free to contact [author](mailto:serhii.s.korol@gmail.com) if you have any idea on how to stabilize it.

Run the following command on MiniPupper to diagnose potential errors in logs:

```shell
docker compose logs -f
```

### ToDo

- [x] Polish FE code
- [ ] Polish BE code
- [x] Add local deployment instructions
- [ ] Add docker-cross builds
- [x] Add teleop and servo control sources
- [ ] Migrate to ROS2
- [ ] Get rid of velocity-smoother, which seems to cause most networking issues
- [ ] Add map for SLAM and navigation
- [x] Integrate this code into MiniPupper repo
