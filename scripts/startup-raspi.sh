#!/bin/bash


cd /home/ubuntu/catkin_ws/
source devel/setup.bash

# please, configure the drone id
roslaunch multi_uav_xbee multi_uav_xbee_network.launch droneId:=0 recordBag:=1