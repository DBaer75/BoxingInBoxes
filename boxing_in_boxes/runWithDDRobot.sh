#!/bin/bash

#create the robot
src/./loadModel.sh

#launch service server and client
sleep 5
roslaunch boxing_in_boxes boxing_in_boxes_launcher.launch

#open the robot controller

#python src/diffDrive_Keyboard_Controller.py