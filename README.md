# Blocks-World

## Introduction:

This is a universty project which use Baxter robot to solve a Blocks World problem using Sense Think Act methodology. 

The project run only as a simulation on Gazebo. 

In this project first we create a world, made of a table and four blocks in different colors using Gazebo simulation. 

Then, we Sense the world using Baxter's head camera and conclude the Initial State (the order of the blocks). 

Afterwards we Think. We do it by creating and solving a planning problem using PDDL. The plannign problem solution is a plan to move the blocks from the given Initial State to a Goal State chosen by the user. 

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

open a new terminal and run: 

`$ rosrun baxter_tools enable_robot.py -e`

`$ rosrun baxter_tools camera_control.py -o head_camera -r 1280x800`

`$ rosrun baxter_interface joint_trajectory_action_server.py`

open another terminal and run: 

`$ roslaunch baxter_moveit_config baxter_grippers.launch`

then run the wanted nodes in another terminal with the command: 

`$ rosrun <package name> <script name>`

first run the `sense.py` node, then the `think_and_act.py` node.  

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
alt="project's video" width="240" height="180" border="10" /></a>

Changing the order of all the blocks: 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ejzN4vBFjZA
" target="_blank"><img src="http://img.youtube.com/vi/ejzN4vBFjZA/0.jpg" 
alt="project's video" width="240" height="180" border="10" /></a>

Switching the red and blue blocks' location:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=U9Tuuu8TSTA
" target="_blank"><img src="http://img.youtube.com/vi/U9Tuuu8TSTA/0.jpg" 
alt="project's video" width="240" height="180" border="10" /></a>

***

In the wiki of thie repo there is a lot of links and information we accumulate during the work on this project. 

