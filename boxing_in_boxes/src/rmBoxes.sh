#for manual deleting of boxes. not used by any other script 
for i in {0..5}
do 
    rosservice call gazebo/delete_model box$i &
    echo $i
done