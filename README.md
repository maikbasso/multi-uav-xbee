# multi_uav_xbee

This packet establishes a network based in position synchronization of multiples UAV nodes.

Run on terminal:
```sh
$ roslaunch multi_uav_xbee multi_uav_xbee_network.launch droneId:=<droneId> serialPort:=<serialPort> baud:=<baud>
```

Default parameters:
 - droneId : 0 # each node with a different ID number
 - serialPort : /dev/ttyUSB0
 - baud : 9600

Published ROS topics:
 - /uav_network/statistics : demonstrates some network statistics.
 - /uav_network/\<droneId\>/pose : local pose of node \<droneId\>.
 - /uav_network/\<droneId\>/set\_pose : wish pose of node \<droneId\>.

This library was been tested on Xbee Pro S1 radio.
