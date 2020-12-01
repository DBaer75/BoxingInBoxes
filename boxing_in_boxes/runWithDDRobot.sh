#!/bin/bash

#create the robot
src/./loadModel.sh

#python src/service_server_box.py

#open the robot controller
#python src/diffDrive_Keyboard_Controller.py


#launch service server and client
roslaunch boxing_in_boxes boxing_in_boxes_launcher.launch

#python src/diffDrive_Keyboard_Controller.py