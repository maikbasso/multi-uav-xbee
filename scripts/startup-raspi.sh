#!/bin/bash


cd /home/ubuntu/catkin_ws/
source /home/ubuntu/catkin_ws/devel/setup.bash

# please, configure the drone id
roslaunch multi_uav_xbee multi_uav_xbee_network.launch droneId:=1 recordBag:=1
