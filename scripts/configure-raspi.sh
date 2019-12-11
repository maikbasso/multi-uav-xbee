#!/bin/bash

sudo sed -i 's/exit/#exit/g' /etc/rc.local
sudo echo "sh /home/$USER/catkin_ws/src/multi-uav-xbee/scripts/startup-raspi.sh &" >> /etc/rc.local
sudo echo "exit 0" >> /etc/rc.local