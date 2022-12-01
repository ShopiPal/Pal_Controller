# Overview
Pal the Robot is a B.Sc Final Project in Mechanical Engineering of the ShopiPal team from Tel-Aviv University (TAU).
The name 'Pal' was given due to the main agenda of the team which says that Robots  are a representation of combining fields of science in order to serve as a friend of the human beings for his own and for his environment benefits.

Pal is a mobile robotic platform capable of autonomously mapping, localizing, and navigating in indoor spaces. Such a platform can be an automated solution that is remotely controlled and useful for applications such as transporting loads from point A to point B and monitoring enclosed spaces such as warehouses, malls and supermarkets.

Main areas of the project were:
- Machanical & HW designing and requirments determination.
- Diffrential driving velocity controller designing and tuning.
- Wrapping scripts into ROS packages and configure it properly.   

# Hardware
The main HW components which the robot contains are:
- Nvidia Jetson Nano - function as Pal 'brain' for high-level tasks and running the robot operating system.
- Arduino Mega micro-controller -  responsible for low-level tasks such as reading the inputs from the robot sensors and send the outputs for the motors according to the logic and algorithms wich occur on the Jetson.  
- DC Motor Driver
- DC motors 
- Ultra-Sonic range sensors
- Motor encoders
- Lidar (Rplidar)
- Wifi module for remote controling (ssh)

The system architecture presented as follow:
![](https://github.com/ShopiPal/Pal_Controller/blob/main/media/System_Architecture.png)


# Usage
The code is spread over 3 Packages: 
- [Pal_Controller](https://github.com/ShopiPal/Pal_Controller/) - the main package of the project, containing the arduino code and the python scripts (rospy nodes) implementing the sensors reading, control loop, subscribing and publishing for the relevant topics.
- [Pal_Navigation](https://github.com/ShopiPal/Pal_Navigation/) - contains the launch and parameters files of existed open-source navigation packages (movebase, amcl, gmapping), subscribing to the relevant topics from wich published on pal_controller
- [Pal_Description](https://github.com/ShopiPal/Pal_Description/) - contains the meshes and URDF file wich exported from SolidWorks.

Assuming: 
- you have the mechanical and HW parts already built and connected properly  
- you have already installed the official updated jetpack (4.x.x series - ubuntu 18) image on the Jetson nano. see: [jetpack installation](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro)
- you have created wifi access point from the Jetson nano wifi module and connect to it from the jetson. see: [wifi instructions](https://forums.developer.nvidia.com/t/can-i-use-intel-wireless-ac8265-wifi-with-jetson-nano-as-internal-wifi-network/212519)
- you have already installed ROS melodic. see : [melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu)
- configured your ROS network on the Robot and on the remote machine. see: [ROS network setup](http://wiki.ros.org/ROS/NetworkSetup)

connect to the robot through ssh and create catkin workspace, clone Pal packages and dependencies (only rplidar until this day) and compile everything: 
```bash
mkdir -p ~/catkin_ws/src
git clone git@github.com:ShopiPal/Pal_Controller.git
git clone git@github.com:ShopiPal/Pal_Navigation.git
git clone git@github.com:ShopiPal/Pal_Description.git
git clone git@github.com:Slamtec/rplidar_ros.git
cd ~/catkin_ws
catkin_make
```

#### Operate Pal: 
First upload ```pal_controller_arduino.ino``` from this package to the arduino using arduino IDE.

Then run pal launch script for running the arduino serial node and the controller:
```bash
roslaunch pal_controller_pkg pal_controller_with_lidar.launch
```
Now the Robot is a live!

For mapping the area use gmapping package:
```bash
roslaunch pal_navigation_pkg pal_gmapping.launch
```
Now you can save the map using ```map_server``` package and the ```map_saver``` command into the [maps](https://github.com/ShopiPal/Pal_Navigation/tree/main/pal_navigation_pkg/maps) directory.

For autonomous navigation use movebase and amcl packages using:
```bash
roslaunch pal_navigation_pkg pal_navigation_planner
```

Use RVIZ to display:
![](https://github.com/ShopiPal/Pal_Controller/blob/main/media/pal_navigat_gif.gif)

### Pal Controller Package 
Nodes: 
- pal_controller_node

Subscribed topics:
- ``` /cmd_vel  ```
- ``` /velocity/vr_current/filter ```
- ``` /velocity/vr_current/raw ```
- ``` /velocity/vl_current/filter ```
- ``` /velocity/vl_current/raw ```
- ``` /delta_distance/dl ```
- ``` /delta_distance/dr```

Published topics:
- ``` /odom  ```
- ``` /left_motor_pwm ```
- ``` /right_motor_pwm ```
- ``` /velocity/vr_target ```
- ``` /velocity/vl_target ```

Services:
- ``` /motors/set_pwm ```
- ``` /motors/stop ```

