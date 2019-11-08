#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from pymobility.models.mobility import random_waypoint

nodeId = int(input("Type the drone id: "))

rospy.init_node('pose_test_publisher_node', anonymous=True)

topicName = "/uav" + str(nodeId) + "/mavros/local_position/pose";
pub = rospy.Publisher(topicName, PoseStamped, queue_size=1)

rw = random_waypoint(1, dimensions=(100, 100), velocity=(0.1, 0.5), wt_max=1.0)

rate = rospy.Rate(1)
while not rospy.is_shutdown():

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    
    positions = next(rw)
	
    goal.pose.position.x = positions[0][0]
    goal.pose.position.y = positions[0][1]
    goal.pose.position.z = 10.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    pub.publish(goal)
    rate.sleep()
