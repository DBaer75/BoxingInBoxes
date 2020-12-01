#rosservice call gazebo/delete_model dd_robot
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/BoxingInBoxes/boxing_in_boxes/models/box.sdf -sdf -model $1 -y $3 -x $2 -z 5.0 


