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
![]()



# Usage

### Pal Controller Package 

# Usefull Links

