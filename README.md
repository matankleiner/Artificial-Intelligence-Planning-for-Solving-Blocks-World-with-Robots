# Blocks-World

## Introduction:

This is a universty project which use Baxter robot to solve Blocks World problem using Sense Think Act methodology. 

The project run only as a simulation on Gazebo. 

In this project first we create a world, made of a table and four blocks in different colors using Gazebo simulation. 

Then, we Sense the world using Baxter's head camera and conclude the Initial State (the order of the blocks). 

Afterwards we Think. We do it by creating and solving a planning problem using PDDL. The plannign problem solution is a plan to move the blocks from the given Initial State to a 

Goal State chosen by the user. 

Then we Act, using Moveit and Baxter and reached the wanted Goal State. 

## Requirements: 

We wrote and ran this project on Ubuntu 16 using [ROS kinetic](http://wiki.ros.org/kinetic/Installation). 

Setup a workstation for Baxter: https://sdk.rethinkrobotics.com/wiki/Workstation_Setup

We used the following programs:

#### [Gazebo](http://gazebosim.org/)

Gazebo is an open-source 3D robotics simulator which using ODE physics engine, OpenGL rendering, and support code for sensor simulation and actuator control.

Baxter simulation for Gazebo: https://sdk.rethinkrobotics.com/wiki/Simulator_Installation

#### [MoveIt](https://moveit.ros.org/)

MoveIt is an open source robotics manipulation platform. We used it features mainly through python code and [Rviz](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html). 

Installation: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html

We used the follwoing python packages (not necessarily at this order, we may have used more packacjes): 

* rospy
* roslib
* requests 
* moveit_commander
* baxter_interface
* cv2 
* scipy 

## How to run the project:

Into a terminal on ubuntu enter the following command: 

`$ ./baxter.sh sim` (after changing directory) 

`$ roslaunch baxter_gazebo baxter_world.launch`

open a new terminal and run: 

`$ rosrun baxter_tools enable_robot.py -e`

`$ rosrun baxter_tools camera_control.py -o head_camera -r 1280x800`

`$ rosrun baxter_interface joint_trajectory_action_server.py`

open another terminal and run: 

`$ roslaunch baxter_moveit_config baxter_grippers.launch`

then run the wanted nodes in another termianl with the command: 

`$ rosrun <package name> <script name>`

##### In case of any error, first try to run the project again. If the script already load models into gazebo (table, blocks) first delete them manually from Gazebo. 


## Team:

Yuval Snir 

Matan Kleinr 

under the guidance of Ronen Nir

***

In the wiki of thie repo there is a lot of links and information we accumulate during the work on this project. 

