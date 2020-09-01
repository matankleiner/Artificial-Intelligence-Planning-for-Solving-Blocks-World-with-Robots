#!/usr/bin/env python

# to use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# this namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
# and a `RobotCommander`_ class. we also import `rospy`_ and some messages that we will use.

import sys
import cProfile, pstats
import time
import rospy
import requests
import roslib

# load_manifest reads the package manifest and sets up the python library path based on the package dependencies. It's required for older rosbuild-based packages, but is no longer needed on catkin.
roslib.load_manifest("moveit_python")

import moveit_commander
import baxter_interface
import geometry_msgs.msg
import rospkg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from copy import deepcopy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Empty, String

# --------------------------------------------------------- INITIALIZATIONS ---------------------------------------------------------
# initialize node
rospy.init_node("pnp")
joint_state_topic = ['joint_states:=/robot/joint_states']  # Yuval added this line (didn't work with sys.arg).
moveit_commander.roscpp_initialize(joint_state_topic)

# instantiate a `MoveGroupCommander` object. this object is an interface
# to a planning group. this interface is used to plan and execute motions.
# we set the group to be the arms of baxter (both, left, right)
# documentation - http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
both_arms = moveit_commander.MoveGroupCommander('both_arms')
right_arm = moveit_commander.MoveGroupCommander('right_arm')
left_arm = moveit_commander.MoveGroupCommander('left_arm')
# enable replanning therefore increase the odds to find a solution
right_arm.allow_replanning(True)
left_arm.allow_replanning(True)
# set the arms' reference frame to assume for poses of end-effectors
right_arm.set_pose_reference_frame('base')
left_arm.set_pose_reference_frame('base')

# baxter_interface is a package
# documentation - http://docs.ros.org/hydro/api/baxter_interface/html/baxter_interface-module.html

# baxter_interface.Gripper is a class - http://docs.ros.org/hydro/api/baxter_interface/html/baxter_interface.gripper.Gripper-class.html
# instantiate a 'Gripper' object. this object allow us to
# control the grippers
leftgripper = baxter_interface.Gripper('left')
rightgripper = baxter_interface.Gripper('right')
# command gripper to open to maximum position
# default values: open(self, block=False, timeout=5.0)
leftgripper.open()
rightgripper.open()

# instantiate a `PlanningSceneInterface` object. this provides a remote interface
# for getting, setting, and updating the robot's internal understanding of the surrounding world.
# base is the fixed frame in which planning is being done
planningScene = PlanningSceneInterface("base")

# --------------------------------------------------------- DEFINES ---------------------------------------------------------

GOAL_X_DELTA = 0.15
MOVEIT_Z_CONST = 0.903
GOAL_MIN_DISTANCE = 0.01
LOCS_MAX_SIZE = 0.24
LOCS_MIN_SIZE = 0.018
MIDDLE_POINT_Z = 0.1
Z_N_INIT_VALUE = -0.143
NUMBER_OF_ATTEMPTS = 6
NUMBER_OF_BLOCKS = 4
# blocks [x,y,z] location, used in load_gazebo_models and in picknplace: (including position 5)
BLOCKS = [[0.55, -0.3, -0.135], [0.55, -0.2, -0.135], [0.55, -0.1, -0.135], [0.55, 0.03, -0.135],
          [0.55 + GOAL_X_DELTA, -0.15, -0.135]]
# table [x, y, z] location, used in load_gazebo_models:
TABLE = [0.7, 0.0, 0.0]
# table size [x, y, z] dimension, used in picknplace:
TABLE_SIZE = [1.0, 0.929999123509, 0.75580154342]
# block size [x, y, z] dimension, used in picknplace:
BLOCK_SIZE = [0.02, 0.02, 0.02]
# center [x, y, z] dimension, used in picknplace:
CENTER = [0.6, -0.0441882472378, -0.52509922829]


# ------------------------------------- INITIAL STATE ----------------------------------------

firstRun = True
## @brief a callback function for the subscriber
def callback(data):
    global firstRun
    if(firstRun):
	firstRun = False
    	# input function
    	goalState = cubes_goal_order()
    	my_data = data.data
    	rospy.loginfo(data.data)
    	initialState = data.data.split()
    	print(initialState)
    	change = 0
    	for i in range(0, len(initialState)):
        	if (initialState[i] != goalState[i]):
            		change = 1
    	# create and solve a planning problem using PDDL
    	if (change):
        	PDDL_planner_n_solver(initialState, goalState)
		solver()
    		picknplace()

## @brief subscribe to "initial_state_from_cameras" topic
## @return return the initial state
def listener():
    #rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber("initial_state_from_cameras", String, callback)
    # spin() keeps python from exiting until this node is stopped - ?
    #rospy.spin()

# ------------------------------------ PLANNING FUNCTION --------------------------------------

## @brief the user enter the way it wants Baxter to organize the blocks
## @return goal_cubes_order the order that the user entered as an array
def cubes_goal_order():
    goal_cubes_order = raw_input(
        "\nEnter the order you want Baxter to organize the blocks, separated by spaces as {red blue green yellow}: ").split()
    for i in range(0, NUMBER_OF_BLOCKS):
        goal_cubes_order[i] = goal_cubes_order[i].lower()
    return goal_cubes_order

## @brief create and solve a PDDL problem
## @param initialState a list of the cubes initial state
## @param goalState a list of the cubes goal state
def PDDL_planner_n_solver(initialState, goalState):
    # create a problem.pddl file based on the initial state and the goal state
    file = open("problem.pddl", "w")
    file.write("(define (problem order_blocksworld)\n\
	(:domain blocksworld)\n")
    file.write("	(:objects   position1 position2 position3 position4 position5\n\
		        red_cube blue_cube green_cube yellow_cube)\n")
    file.write("	(:init  (position position1)\n\
			(position position2)\n\
			(position position3)\n\
			(position position4)\n\
			(position position5)\n\
	        	(cube red_cube)\n\
			(cube blue_cube)\n\
			(cube green_cube)\n\
			(cube yellow_cube)\n")
    file.write("			(at red_cube    position" + str(initialState.index("red") + 1) + ")\n")
    file.write("			(at blue_cube    position" + str(initialState.index("blue") + 1) + ")\n")
    file.write("			(at green_cube    position" + str(initialState.index("green") + 1) + ")\n")
    file.write("			(at yellow_cube    position" + str(initialState.index("yellow") + 1) + ")\n")
    file.write("			(empty position5))\n\n")
    file.write("	(:goal (and (at red_cube    position" + str(goalState.index("red") + 1) + ")\n")
    file.write("			(at blue_cube    position" + str(goalState.index("blue") + 1) + ")\n")
    file.write("			(at green_cube    position" + str(goalState.index("green") + 1) + ")\n")
    file.write("			(at yellow_cube    position" + str(goalState.index("yellow") + 1) + ")))\n")
    file.write(")")
    file.close

def solver():
	data = {'domain': open('domain.pddl', 'r').read(),'problem': open('problem.pddl', 'r').read()}
	resp = requests.post('http://solver.planning.domains/solve',verify=False, json=data).json()
	with open('plan.ipc', 'w') as f:
    		f.write('\n'.join([act['name'] for act in resp['result']['plan']]))


# ---------------------------------------------------- MOVE FUNCTION ----------------------------------------------------

## @brief add the current position as the first point for a movement. called upon 1 time from move function
## @param arm which arm the robot will use
## @param *arg pass waypoints to the function as a list
## @return current_pos the current position of the specified arm as a Pose - geometry msg
def set_current_position(arm, *arg):
    if (arm == 'left'):
        # get the current pose of the end-effector of the group.
        # throws an exception if there is not end-effector.
        current_position = left_arm.get_current_pose()
    if (arm == 'right'):
        current_position = right_arm.get_current_pose()
    """
    geometry_msgs is a package that provides messages for common
    geometric primitives such as points, vectors, and poses.
    instantiate a 'Pose' object. provide a representation of pose in free space,
    composed of position and orientation - Point position and Quaternion orientation
    documentation - http://docs.ros.org/melodic/api/geometry_msgs/html/index-msg.html
    Quaternion - https://en.wikipedia.org/wiki/Quaternion -----> a way to describe points in space
    position - contains the position of a point in free space as (x, y, z) all as float64. 
    orientation - represents an orientation in free space in quaternion form as (x, y, z, w) all as float64. 
    """
    current_pos = geometry_msgs.msg.Pose()
    current_pos.position.x = current_position.pose.position.x
    current_pos.position.y = current_position.pose.position.y
    current_pos.position.z = current_position.pose.position.z
    current_pos.orientation.x = current_position.pose.orientation.x
    current_pos.orientation.y = current_position.pose.orientation.y
    current_pos.orientation.z = current_position.pose.orientation.z
    current_pos.orientation.w = current_position.pose.orientation.w

    i = len(arg)
    # only one argument has been send to function, and because it is always
    # like this (in the one time the function has been called) this if will always happen
    if (i == 1):
        waypoints = arg[0]
        waypoints.append(current_pos)
    return current_pos


## @brief compute and execute a path to goal. if the cartesian path fail or not accurate enough
##        try an ordinary path to goal.
##        the function is limited to 3 waypoints but more can be added.
##        called upon 4 times from picknplace function, only with 'right' limb
## @param arm which arm the robot will use
## @param *arg a list of waypoints
def move(arm, *arg):
    fraction = 0
    attempts = 0
    waypoints = []
    set_current_position(arm, waypoints)
    i = len(arg)  # i is the number of waypoints.
    waypoints.append(arg[0])
    # "goal" is the end position of the movement, if there are more than 1 point it will contain the last one.
    goal = arg[0]
    goal_x = goal.position.x
    goal_y = goal.position.y
    goal_z = goal.position.z
    if (i > 1):
        goal = arg[1]
        goal_x = goal.position.x
        goal_y = goal.position.y
        goal_z = goal.position.z
        waypoints.append(arg[1])
    if (i > 2):
        goal = arg[2]
        goal_x = goal.position.x
        goal_y = goal.position.y
        goal_z = goal.position.z
        waypoints.append(arg[2])

    if (arm == 'right'):
        right_arm.set_start_state_to_current_state()
        """
        we use the following function: 
        def moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(self,
 	                                                                          waypoints,
 	                                                                          eef_step,
 	                                                                          jump_threshold,
 	                                                                          avoid_collisions = True
 	                                                                          )
 	    the function compute a sequence of waypoints that make the end-effector move in straight line
        segments that follow the poses specified as waypoints.
        configurations are computed for every eef_step meters;
        the jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath.
        The return value is a tuple: a fraction of how much of the path was followed (e.g: 0.95.454545 -> followed 95.454545% of requested trajectory)
	    the actual RobotTrajectory    
	    we want that the cartesian path will be interpolated at a resolution of 0.1 cm -> eef_step in cartesian translation is 0.001.
        the jump threshold is specify as 0.0, effectively disabled.                                                                       		
        """
        (plan, fraction) = right_arm.compute_cartesian_path(waypoints, 0.001, 0.0, True)
        # execute a previously planned path - in this case "plan"
        right_arm.execute(plan, wait=True)
        # read the position of the right arm to compare it with the goal.
        right_arm_pose = right_arm.get_current_pose()  # a is a not meaningful name!
        x_pos = right_arm_pose.pose.position.x
        y_pos = right_arm_pose.pose.position.y
        z_pos = right_arm_pose.pose.position.z
        # while the position of the end-effector is too far from goal position
        while not ((abs(z_pos - goal_z) < 0.01) and (abs(y_pos - goal_y) < 0.01) and (abs(x_pos - goal_x) < 0.01)):
            right_arm_pose = right_arm.get_current_pose()
            x_pos = right_arm_pose.pose.position.x
            y_pos = right_arm_pose.pose.position.y
            z_pos = right_arm_pose.pose.position.z
            time.sleep(1)
            # in case the cartesian path failed compute a normal path
            if (attempts > NUMBER_OF_ATTEMPTS):
                print("----->cartesian path failed!<-----")  # print to a terminal
                right_arm.set_pose_target(goal)  # set the pose of the end-effector
                right_arm.plan()  # return a motion plan (a RobotTrajectory) to the set goal state.
                # the right will arm move to the goal position. it'll continue after it finished.
                right_arm.go(wait=True)  # set the target of the group and then move the group to the specified target
            attempts += 1

    # the same methods in case the left arm will be called
    if (arm == 'left'):
        left_arm.set_start_state_to_current_state()
        (plan, fraction) = left_arm.compute_cartesian_path(waypoints, 0.001, 0.0, True)
        left_arm.execute(plan, wait=True)
        left_arm_pose = left_arm.get_current_pose()  # a is still not meaningful name!
        x_pos = left_arm_pose.pose.position.x
        y_pos = left_arm_pose.pose.position.y
        z_pos = left_arm_pose.pose.position.z
        while not ((abs(z_pos - goal_z) < 0.01) and (abs(y_pos - goal_y) < 0.01) and (abs(x_pos - goal_x) < 0.01)):
            left_arm_pose = left_arm.get_current_pose()
            x_pos = left_arm_pose.pose.position.x
            y_pos = left_arm_pose.pose.position.y
            z_pos = left_arm_pose.pose.position.z
            time.sleep(1)

            if (attempts > NUMBER_OF_ATTEMPTS):
                print("----->cartesian path failed!<-----")
                left_arm.set_pose_target(goal)
                left_arm.plan()
                left_arm.go(wait=True)
            attempts += 1


# -------------------------------------------------------- PICK AND PLACE --------------------------------------------------------

## @brief perform the pick and place action. called upon from main
def picknplace():
    # define the initial positions of baxter's joints in a dictionary
    initial_position = {'left_e0': -1.69483279891317, 'left_e1': 1.8669726956453, 'left_s0': 0.472137005716569,
                        'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143,
                        'left_w2': -0.6339059781326424, 'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506,
                        'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871,
                        'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}

    # create two lists, one of initial locations and one of goal locations
    # for BAXTER to move them from one to another
    locs_x = [] # initial locations
    locs_y = []
    orien = []
    goal_move = []
    place_goal = [] # goal locations

    # using readlines() to read the plan.ipc file, the PDDL problem solution
    PDDL_solved = open('plan.ipc', 'r')
    Lines = PDDL_solved.readlines()
    # adding the goal positions from the PDDL solved file
    for line in Lines:
        orien.append(0);
        goal_move.append(line.split()[3][8:9])
        cord_dest = int(line.split()[2][8:9])
        # adding an offset to all location but the 5th
        if(cord_dest != 5):
        	locs_x.append(BLOCKS[cord_dest - 1][0] + 0.025)
        	locs_y.append(BLOCKS[cord_dest - 1][1] + 0.03)
	else:
		locs_x.append(BLOCKS[cord_dest - 1][0])
        	locs_y.append(BLOCKS[cord_dest - 1][1])

    for i in goal_move:
        place_goal_i = geometry_msgs.msg.Pose()
        if(int(i) != 5):
        	place_goal_i.position.x = BLOCKS[int(i) - 1][0] + 0.025
        	place_goal_i.position.y = BLOCKS[int(i) - 1][1] + 0.03
        else:
		place_goal_i.position.x = BLOCKS[int(i) - 1][0]
        	place_goal_i.position.y = BLOCKS[int(i) - 1][1]
        place_goal_i.position.z = BLOCKS[int(i) - 1][2]
        place_goal_i.orientation.x = 1.0
        place_goal_i.orientation.y = 0.0
        place_goal_i.orientation.z = 0.0
        place_goal_i.orientation.w = 0.0
        place_goal.append(place_goal_i)

    # the distance from the zero point in Moveit to the ground is 0.903 m (for the table we use)
    # the value is not always the same. (look in Readme)
    center_z_cube = -MOVEIT_Z_CONST + TABLE_SIZE[2] + BLOCK_SIZE[2] / 2

    # initialize a list for the objects and the stacked cubes.
    objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']

    both_arms.set_joint_value_target(initial_position)
    both_arms.plan()
    both_arms.go(wait=True)

    # remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)
    # wait for All Clear message from emulator startup.
    rospy.wait_for_message("/robot/sim/started", Empty)

    planningScene.clear()
    """
    Attach a box into the planning scene.
    def moveit_python.planning_scene_interface.PlanningSceneInterface.attachBox	(self,
 	                                                                         name,               - Name of the object
 	                                                                         size_x,             - The x-dimensions size of the box
 	                                                                         size_y,             - The y-dimensions size of the box
 	                                                                         size_z,             - The z-dimensions size of the box
                                                                                 x,                  - The x position in link_name frame
                                                                                 y,                  - The y position in link_name frame
                                                                                 z,                  - The z position in link_name frame
                                                                                 link_name,          - Name of link to attach this object to
                                                                                 touch_links = None, - Names of robot links that can touch this object
                                                                                 detach_posture = None,
                                                                                 weight = 0.0,
                                                                                 wait = True         - When true, we wait for planning scene to actually
                                                                                                       update, this provides immunity against lost messages.
                                                                                 )		    
    add the table as attached object
    """
    planningScene.attachBox('table', TABLE_SIZE[0], TABLE_SIZE[1], TABLE_SIZE[2], CENTER[0], CENTER[1], CENTER[2],
                            'base', touch_links=['pedestal'])
    planningScene.waitForSync()
    rightgripper.open()

    # cProfile explanation - https://docs.python.org/2/library/profile.html#module-cProfile
    # cProfile to measure the performance (time) of the task.
    profile = cProfile.Profile()
    profile.enable()

    # loop performing "pick and place" till all objects are cleared from table.
    # locs_x,locs_y are the source fot he cubes.
    num_of_pick_n_place = 0
    while num_of_pick_n_place < len(goal_move):

        # do the task only if there are still objects on the table
        moved_objects = 0
        while moved_objects < len(locs_x):
            # clear planning scene
            planningScene.clear()
            # Add table as attached object.
            planningScene.attachBox('table', TABLE_SIZE[0], TABLE_SIZE[1], TABLE_SIZE[2], CENTER[0], CENTER[1],
                                    CENTER[2], 'base', touch_links=['pedestal'])

            # Initialize the data of the objects on the table.
            x_n = locs_x[moved_objects]
            y_n = locs_y[moved_objects]
            z_n = Z_N_INIT_VALUE
            th_n = orien[moved_objects]
            # if the angle is bigger than 45 degrees, I assume there will be problem with the pick,
            # therefore in that case we need to calibrate the angle to be in the range[-45,45] degrees.
            # I surmise the offset of the angle is always adding toward the positive axis for theta,
            # thus we need to calibrate theta toward the negative 45.
            if th_n > pi / 4:
                th_n = -1 * (th_n % (pi / 4))

            # Add the detected objects into the planning scene.
            for i in range(1, len(locs_x)):
                planningScene.addBox(objlist[i], BLOCK_SIZE[0], BLOCK_SIZE[1], BLOCK_SIZE[2], locs_x[i], locs_y[i],
                                     center_z_cube)
            planningScene.waitForSync()

            # defien a middle point from initial point to goal, as "approach_pick_goal".
            # initalize it as equal to the actual pick_goal in z and y axis, and different in z axis
            approach_pick_goal = geometry_msgs.msg.Pose()
            approach_pick_goal.position.x = x_n
            approach_pick_goal.position.y = y_n
            approach_pick_goal.position.z = z_n + MIDDLE_POINT_Z

            # PoseStamped is a Pose with reference coordinate frame and timestamp
            approach_pick_goal_dummy = PoseStamped()
            """"
            Header is  Standard metadata for higher-level stamped data types.
            This is generally used to communicate timestamped data in a particular coordinate frame.
            sequence ID: consecutively increasing ID  uint32 seq
            Two-integer timestamp that is expressed as:
            * stamp.sec: seconds (stamp_secs) since epoch (called 'secs')
            * stamp.nsec: nanoseconds since stamp_secs (called 'nsecs')
            """
            approach_pick_goal_dummy.header.frame_id = "base"
            approach_pick_goal_dummy.header.stamp = rospy.Time.now()
            approach_pick_goal_dummy.pose.position.x = x_n
            approach_pick_goal_dummy.pose.position.y = y_n
            approach_pick_goal_dummy.pose.position.z = z_n + MIDDLE_POINT_Z
            approach_pick_goal_dummy.pose.orientation.x = 1.0
            approach_pick_goal_dummy.pose.orientation.y = 0.0
            approach_pick_goal_dummy.pose.orientation.z = 0.0
            approach_pick_goal_dummy.pose.orientation.w = 0.0

            # orientate the gripper --> uses function from geometry.py (by Mike Ferguson) to 'rotate a pose' given rpy angles.
            approach_pick_goal_dummy.pose = rotate_pose_msg_by_euler_angles(approach_pick_goal_dummy.pose, 0.0, 0.0,
                                                                            th_n)
            approach_pick_goal.orientation.x = approach_pick_goal_dummy.pose.orientation.x
            approach_pick_goal.orientation.y = approach_pick_goal_dummy.pose.orientation.y
            approach_pick_goal.orientation.z = approach_pick_goal_dummy.pose.orientation.z
            approach_pick_goal.orientation.w = approach_pick_goal_dummy.pose.orientation.w

            # Move to the approach goal and the pick_goal.
            pick_goal = deepcopy(approach_pick_goal)  # create an exact copy of approach_pick_goal
            pick_goal.position.z = z_n
            move('right', approach_pick_goal)  # move is a function
            time.sleep(5)
            move('right', pick_goal)
            time.sleep(1)
            rightgripper.close()  # PICK!
            time.sleep(1)
            # move back to the approach_pick_goal.
            move('right', approach_pick_goal)

            # define the approach_place_goal
            approached_place_goal = deepcopy(place_goal[num_of_pick_n_place])
            approached_place_goal.position.z = approached_place_goal.position.z + MIDDLE_POINT_Z
            move('right', approached_place_goal)
            time.sleep(5)
            place_goal_val = place_goal[num_of_pick_n_place]
            place_goal_val.position.z = place_goal_val.position.z + BLOCK_SIZE[2]
            move('right', place_goal_val)
            time.sleep(1)
            rightgripper.open()  # PLACE!
            time.sleep(1)
            # move to the approach_place_goal.
            move('right', approached_place_goal)
            # increase iterators
            num_of_pick_n_place += 1
            moved_objects += 1
            # move the arms to initial position.
            both_arms.set_joint_value_target(initial_position)
            both_arms.plan()
            both_arms.go(wait=True)

    profile.disable()
    # pstats documentation - https://docs.python.org/3/library/profile.html#module-pstats
    pstats.Stats(profile).sort_stats('cumulative').print_stats(0.0)

    # exit MoveIt and shutting down the process
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    time.sleep(10)


# -------------------------------------------- MAIN --------------------------------------------

def main():
    # load gazebo models via spawning services
    listener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt was detected")
    return 0

if __name__ == '__main__':
    sys.exit(main())
