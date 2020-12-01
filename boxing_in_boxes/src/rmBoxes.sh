for i in {0..5}
do 
    rosservice call gazebo/delete_model box$i &
    echo $i
done