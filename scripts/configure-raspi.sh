#!/bin/bash

mkdir -p /home/ubuntu/catkin_ws/src/multi-uav-xbee/bag

sudo sed -i 's/exit/#exit/g' /etc/rc.local
sudo sed -i 's/source/#source/g' /etc/rc.local
sudo echo "source /home/ubuntu/catkin_ws/src/multi-uav-xbee/scripts/startup-raspi.sh &" >> /etc/rc.local
sudo echo "exit 0" >> /etc/rc.local