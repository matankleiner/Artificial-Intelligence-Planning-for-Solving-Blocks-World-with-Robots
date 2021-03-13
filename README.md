# Artificial Intelligence Planning for Solving Blocks World with Robots 

[![shield](https://img.shields.io/badge/ros-kinetic-yellow)](http://wiki.ros.org/kinetic)
[![shield](https://img.shields.io/badge/language-python-blue)](https://www.python.org/)
[![shield](https://img.shields.io/badge/module-rospy-green)](http://wiki.ros.org/rospy)
[![shield](https://img.shields.io/badge/module-cv2-green)](https://opencv.org/)
[![shield](https://img.shields.io/badge/module-moveit-green)](http://wiki.ros.org/moveit)

![alt text](https://github.com/matankleiner/Artificial-Intelligence-Planning-for-Solving-Blocks-World-with-Robots/blob/master/images/vlcsnap-2020-09-07-15h46m08s601.png)

## Introduction:

This is a university project which use Baxter robot to solve a Blocks World problem using **Sense Think Act** methodology. 

The project run only as a simulation on Gazebo. 

In this project first we create a world, made of a table and four blocks in different colors using Gazebo simulation. 

Then, we **Sense** the world using Baxter's head camera and conclude the Initial State (the order of the blocks). 

Afterwards we **Think**. We do it by creating and solving a planning problem using PDDL. The plannign problem solution is a plan to move the blocks from the given Initial State to a Goal State chosen by the user. 

Then we **Act**, using Moveit and Baxter and reached the wanted Goal State. 

![alt text](https://github.com/matankleiner/Artificial-Intelligence-Planning-for-Solving-Blocks-World-with-Robots/blob/master/images/sense%20think%20act%20diagram.png)

## Requirements: 

We wrote and ran this project on Ubuntu 16 using [ROS kinetic](http://wiki.ros.org/kinetic/Installation). 

Setup a workstation for Baxter: https://sdk.rethinkrobotics.com/wiki/Workstation_Setup

We used the following programs:

#### [Gazebo](http://gazebosim.org/)

Gazebo is an open-source 3D robotics simulator which using ODE physics engine, OpenGL rendering, and support code for sensor simulation and actuator control.

Baxter simulation for Gazebo: https://sdk.rethinkrobotics.com/wiki/Simulator_Installation

![alt text](https://github.com/matankleiner/Artificial-Intelligence-Planning-for-Solving-Blocks-World-with-Robots/blob/master/images/Gazebo.png)

#### [MoveIt](https://moveit.ros.org/)

MoveIt is an open source robotics manipulation platform. We used it features mainly through python code and [Rviz](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html). 

Installation: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html

![alt text](https://github.com/matankleiner/Artificial-Intelligence-Planning-for-Solving-Blocks-World-with-Robots/blob/master/images/Rviz.png)

We used the follwoing external python packages (not necessarily at this order, we may have used more packages): 

* rospy
* roslib
* roscpp
* rospack
* gazebo_msgs
* gazebo_ros
* sensor_msgs
* std_msgs
* visualization_msgs
* moveit_commander
* baxter_core_msgs
* baxter_interface
* baxter_gazebo
* baxter_tools
* cv2 
* cv_bridge
* scipy
* requests 

## How to run the project:

Into a terminal on ubuntu enter the following command: 

`$ ./baxter.sh sim` (make sure you are in the directory where baxter.sh file is) 

`$ roslaunch baxter_gazebo baxter_world.launch`

Open a new terminal and run: 

`$ rosrun baxter_tools enable_robot.py -e`

`$ rosrun baxter_tools camera_control.py -o head_camera -r 1280x800`

`$ rosrun image_view image_view image:=/cameras/head_camera/image`

Open another terminal and run: 

`$ rosrun baxter_interface joint_trajectory_action_server.py`

Open another terminal and run: 

`$ roslaunch baxter_moveit_config baxter_grippers.launch`

Then run the wanted nodes in another terminal with the command: 

`$ rosrun <package name> <script name>`

First run the `sense.py` node, then the `think_and_act.py` node, each one in a different terminal.  

You should run the `think_and_act.py` script from the PDDL directory.

##### In case of any error, first try to run the project again.

##### If the script already load models into gazebo (table, blocks) first delete them manually from Gazebo. 


## Team:

Yuval Snir 

Matan Kleiner 

under the guidance of Ronen Nir

## Project's Video: 

Video summing the project: 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=US_xKeu5Bwk
" target="_blank"><img src="http://img.youtube.com/vi/US_xKeu5Bwk/0.jpg" 
alt="project's video" width="360" height="240" border="10" /></a>

Changing the order of all the blocks: 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ejzN4vBFjZA
" target="_blank"><img src="http://img.youtube.com/vi/ejzN4vBFjZA/0.jpg" 
alt="project's video" width="360" height="240" border="10" /></a>

Switching the location of each pair of blocks:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Jw--K7xer00
" target="_blank"><img src="http://img.youtube.com/vi/Jw--K7xer00/0.jpg" 
alt="project's video" width="360" height="240" border="10" /></a>

Switching the red and blue blocks' location:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=U9Tuuu8TSTA
" target="_blank"><img src="http://img.youtube.com/vi/U9Tuuu8TSTA/0.jpg" 
alt="project's video" width="360" height="240" border="10" /></a>

***

In the [wiki](https://github.com/matankleiner/Artificial-Intelligence-Planning-for-Solving-Blocks-World-with-Robots/wiki) of thie repo there is a lot of links and information we accumulate during the work on this project. 

