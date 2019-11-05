# Multi-Robot Collaboratio with two UR3s
## Project Overview
The goal of this project was to have two UR3 robots tossing a balloon between one another once the latter had been introduced into the environment by a person. A GUI (Graphical User Interface) was developed to interface with the real robot and the simulation along with other features (collision detection/avoidance, Emergency Stop button, robot movement in joint & Cartesian space, etc.). Various concepts were used for performing including but not limited to forward & inverse kinematics, joystick control, RMRC (Resolved Motion Rate Control), image processing & visual servoing (with a Kinect v2 camera).

The project was the result of group work. The program was written in MATLAB and executed on Ubuntu. Additionally, OpenCV and Python were used to determine the balloon centre’s position in Cartesian space and ROS was used to interact with the robot.

##ROS Packages
In order to run this code, the following ROS packages had to be installed:
- opencv2
- iai_kinect2

## Authors
- Ahmad Kamal
- Joshua Shallita
- Rebecca Carrucan
