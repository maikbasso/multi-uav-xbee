#!/bin/bash

sudo sed -i 's/sh /home/$USER/catkin_ws/src/multi-uav-xbee/scripts/startup-raspi.sh &/exit 0/g' /etc/rc.local 
sudo echo "exit 0" >> /etc/rc.local