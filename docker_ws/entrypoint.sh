#!/bin/bash
set -e
 
# Carica l'installazione base di ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
 
# Carica il workspace locale se è stato compilato
if [ -f "/ros_ws/install/setup.bash" ]; then
    source /ros_ws/install/setup.bash
fi
 
# Configurazione variabili per Waffle e Gazebo
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export GZ_SIM_RESOURCE_PATH=/ros_ws/src:$GZ_SIM_RESOURCE_PATH
export DISPLAY=${DISPLAY:-:0}
 
exec "$@"