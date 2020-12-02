#delete the robot if it exists and then spawn the robot
rosservice call gazebo/delete_model DD_robot
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/BoxingInBoxes/boxing_in_boxes/models/model.sdf -sdf -model DD_robot -y 0.0 -x -0.0 -z 1.0
