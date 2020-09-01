#!/usr/bin/env python
from __future__ import print_function

#import colorsys
#import json
#import numpy

import roslib
roslib.load_manifest('think_and_act')
import sys
import rospy
import rospkg
import cv2 # openCV for image processing 
# messeges for subscribing and publishing
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # cv_bridge bridges the gap between ros msgs and openCV
from enum import Enum
from scipy import ndimage # import scipy library for center of mass calculation
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# blocks [x,y,z] location, used in load_gazebo_models and in picknplace: (including position 5)
GOAL_X_DELTA = 0.15
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


# -------------------------------------- GAZEBO MODELS --------------------------------------------

## @brief load and spawn table's SDF file and blocks' URDF file into to the world to create the INITIAL STATE
def load_gazebo_models(table_pose=Pose(position=Point(x=TABLE[0], y=TABLE[1], z=TABLE[2])),
                       table_reference_frame="world",
                       block_pose1=Pose(position=Point(x=BLOCKS[0][0], y=BLOCKS[0][1], z=BLOCKS[0][2])),
                       block_pose2=Pose(position=Point(x=BLOCKS[1][0], y=BLOCKS[1][1], z=BLOCKS[1][2])),
                       block_pose3=Pose(position=Point(x=BLOCKS[2][0], y=BLOCKS[2][1], z=BLOCKS[2][2])),
                       block_pose4=Pose(position=Point(x=BLOCKS[3][0], y=BLOCKS[3][1], z=BLOCKS[3][2])),
                       block_reference_frame="base"):
    # get models' path
    model_path = rospkg.RosPack().get_path('think_and_act') + "/models/"
    # load table SDF
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')
    # load BLOCK1 (red) URDF
    block_xml1 = ''
    with open(model_path + "block/model1.urdf", "r") as block_file:
        block_xml1 = block_file.read().replace('\n', '')
    # load BLOCK2 (blue) URDF
    block_xml2 = ''
    with open(model_path + "block/model2.urdf", "r") as block_file:
        block_xml2 = block_file.read().replace('\n', '')
    # load BLOCK3 (green) URDF
    block_xml3 = ''
    with open(model_path + "block/model3.urdf", "r") as block_file:
        block_xml3 = block_file.read().replace('\n', '')
    # load BLOCK3 (yellow) URDF
    block_xml4 = ''
    with open(model_path + "block/model4.urdf", "r") as block_file:
        block_xml4 = block_file.read().replace('\n', '')
    # spawn table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # spawn BLOCK1 (red) URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml1, "/",
                               block_pose1, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # spawn BLOCK2 (blue) URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block2", block_xml2, "/",
                               block_pose2, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # spawn BLOCK3 (green) URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block3", block_xml3, "/",
                               block_pose3, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # spawn BLOCK4 (yellow) URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block4", block_xml4, "/",
                               block_pose4, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


## @brief delete the gazebo models from world. will be called when shutdown has occurred
def delete_gazebo_models():
    print("--------------->shutdown function is called<---------------")  # print to a terminal
    """
    there is an internal bug in gazebo, sometimes
    "rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)"
    won't work properly. to bypass this bug we call a new serviceProxy
    for every model we want to delete via for loop. 
    link to describe the bug: 
    https://get-help.robotigniteacademy.com/t/ros-services-part-1-gazebo-delete-model-service-does-not-work-gazebo-model-states-no-longer-working-after-calling-the-service/736
    """
    delete_models = ["cafe_table", "block1", "block2", "block3", "block4"]
    for i in delete_models:
        # call to a DeleteModel service than delete a model
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(i)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

# ----------------------------------------------------------------------------------------------------------

class image_to_initial_state:
    received_image = [None]

    class Color(Enum):
        Red = 0
        Blue = 1
        Green = 2
        Yellow = 3

    ## @brief initialize the class artifacts
    def __init__(self):
        self.initial_state_pub = rospy.Publisher("initial_state_from_cameras", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callback)
        self.received_image = [None]
        # dictionary for colors' lower and upper HSV values
        self.ColorDict = {self.Color.Red: {'LowerHSV': (0, 70, 50), 'UpperHSV': (10, 255, 255)},
                          self.Color.Blue: {'LowerHSV': (110, 50, 50), 'UpperHSV': (130, 255, 255)},
                          self.Color.Green: {'LowerHSV': (50, 100, 50), 'UpperHSV': (70, 255, 255)},
                          self.Color.Yellow: {'LowerHSV': (20, 100, 90), 'UpperHSV': (50, 255, 255)}}
        self.Cubes_center_of_mass = [[0, 0], [0, 0], [0, 0], [0, 0]]

    ## @brief ROS saves images as ros messages, therefore we need to convert it
    ##        using CvBridge to cv2 format in order to do image processing
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("Error! image converting to cv2 format failed")
        self.received_image[0] = cv_image

    ## @brief creates the masked image for each cube specific color
    ## @param Color an Enum of the used colors
    ## @return mask a masked image
    def image_processing(self, Color):
        original = self.received_image[0]
        # converts the image to HSV
        hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
        # a different image processing routine for red due to Baxter's red body
        if Color == self.Color.Red:
            # splits the image to h, s, v where v is a gray scale image
            (h, s, v) = cv2.split(hsv)
            # turns all the pixels that greater than 600 in y axis to black (WORK FOR HEAD CAMERA!!!)
            v[540:][:] = 1
            # merges the three matrices back to hsv format
            hsv_no_red = cv2.merge((h, s, v))
            # filters according to the HSV values
            mask = cv2.inRange(hsv_no_red, self.ColorDict.get(Color).get('LowerHSV'),
                               self.ColorDict.get(Color).get('UpperHSV'))
        else:
            # filters according to the HSV values
            mask = cv2.inRange(hsv, self.ColorDict.get(Color).get('LowerHSV'),
                               self.ColorDict.get(Color).get('UpperHSV'))
        # erode and dilate the mask in order to clean noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    ## @brief calculates the center of mass of the cubes using a masked image of each cube
    def masked_image(self):
        red_mask = self.image_processing(self.Color.Red)
        (self.Cubes_center_of_mass[self.Color.Red.value][1],
         self.Cubes_center_of_mass[self.Color.Red.value][0]) = ndimage.measurements.center_of_mass(red_mask)
        blue_mask = self.image_processing(self.Color.Blue)
        (self.Cubes_center_of_mass[self.Color.Blue.value][1],
         self.Cubes_center_of_mass[self.Color.Blue.value][0]) = ndimage.measurements.center_of_mass(blue_mask)
        green_mask = self.image_processing(self.Color.Green)
        (self.Cubes_center_of_mass[self.Color.Green.value][1],
         self.Cubes_center_of_mass[self.Color.Green.value][0]) = ndimage.measurements.center_of_mass(green_mask)
        yellow_mask = self.image_processing(self.Color.Yellow)
        (self.Cubes_center_of_mass[self.Color.Yellow.value][1],
         self.Cubes_center_of_mass[self.Color.Yellow.value][0]) = ndimage.measurements.center_of_mass(yellow_mask)
        

    ## @brief calculate the initial state from the center of mass and publish it
    def display_initial_state(self):
        self.masked_image()
	center_of_mass = self.Cubes_center_of_mass
        x_center_of_mass = []
        sorted_x_center_of_mass = []
        for i in range(0,4):
            x_center_of_mass.append(int(center_of_mass[i][0]))
	    sorted_x_center_of_mass.append(int(center_of_mass[i][0]))
        sorted_x_center_of_mass.sort()
        initial_state = []
        colors = ["red", "blue", "green", "yellow"]
        for i in sorted_x_center_of_mass:
            initial_state.append(colors[x_center_of_mass.index(i)])
	initial_state.reverse()
	str_to_publish = ""
        for i in initial_state:
		str_to_publish = str_to_publish + i + " "
        print(str_to_publish)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo(str_to_publish)
            self.initial_state_pub.publish(str_to_publish)
            rate.sleep()

    ## @brief safe shutdown
    def shutdown(self):
        print("Shutting down safely")
	delete_gazebo_models()

def shutdown():
        delete_gazebo_models()
        print("Shutting down safely")

def main(args):
    load_gazebo_models()
    i2initialState = image_to_initial_state()
    # we use "disable_signals" because ROS and OpenCV  try to control the same signals,
    # so it's necessary to disable it on ROS to avoid conflicts.
    rospy.init_node("image_to_initial_state", disable_signals=True, anonymous=True)

    i2initialState.display_initial_state()

    try:
   	rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt was detected")

    # safe shutdown
    rospy.on_shutdown(i2initialState.shutdown)

if __name__ == '__main__':
    main(sys.argv)
