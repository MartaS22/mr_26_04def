xhost +local:root > /dev/null 2>&1 || xhost +
 
echo "Avvio del container ROS 2..."  
docker run -it --rm --net host --ipc host --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/.Xauthority:/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -e QT_X11_NO_MITSHM=1 \
    -e TURTLEBOT3_MODEL=waffle \
    -v "$(pwd)/ros_ws:/ros_ws" \
    --name="ros2_maze_container" \
    ros2_maze_explorer bash