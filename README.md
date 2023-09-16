# General Purpose Robot
Autonomous robot utilizing ROS2 and Open CV.

# Software
Python, C++, XML, CMake, Ubuntu 22.04.3, ROS2 Humble, Open CV

# Hardware
Raspberry Pi 4b, Arduino Uno Rev3, Arduino Nano Every, LD19 360 degrees 2D Lidar, ultrasonic sensors, camera module, 11.9 inch LCD, stepper motors, omniwheels, cnc shield, servo hat, LEDs

# Arduino
The two arduino code files are implemented on the Arduino Uno and the Arduino Nano respectively.

# Custom Interfaces
A custom msg interface used for publishing/subscribing ultrasonic sensor data.

# Main Package
The main package contains all of the ROS2 nodes created for running the different robotic components and sharing data between them.

# Robot Bringup
This file allows us do start up the robot using only a single terminal command.
