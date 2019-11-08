#!/usr/bin/env python

# Import required Python code.
import rospy
from geometry_msgs.msg import Pose
from drawnow import drawnow, figure
import matplotlib.pyplot as plt
import numpy as np
import tf

my_dpi=96
fig = plt.figure(figsize=(480/my_dpi, 480/my_dpi), dpi=my_dpi)

maxNumberOfNodes = 10
nodePose = np.zeros((maxNumberOfNodes,6))

ID_X = 0
ID_Y = 1
ID_Z = 2
ID_Roll = 3
ID_Pitch = 4
ID_Yaw = 5

FIELD_X = 100
FIELD_Y = 100

def callback(data):
    topicName = data._connection_header["topic"]

    test = ""

    startPos = topicName.find('uav_network')
    firstPos = topicName.find('/', startPos)
    lastPos = topicName.find('/', firstPos+1)

    for i in range(firstPos+1, lastPos):
            test=test + topicName[i]

    droneId = int(test)

    nodePose[droneId][ID_X] = data.position.x
    nodePose[droneId][ID_Y] = data.position.y
    nodePose[droneId][ID_Z] = data.position.z

    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    nodePose[droneId][ID_Roll]  = euler[0]
    nodePose[droneId][ID_Pitch] = euler[1]
    nodePose[droneId][ID_Yaw]   = euler[2]


def PlotPosition2D():
    plt.xlim(0, FIELD_X)
    plt.ylim(0, FIELD_Y)
    plt.scatter(nodePose[:,ID_X], nodePose[:,ID_Y], c="y", edgecolors='face')

if __name__ == '__main__':
    rospy.init_node('position_graph_node', anonymous = True)

    for i in xrange(maxNumberOfNodes):
        topicName = "/uav_network/" + str(i) + "/pose"
        rospy.Subscriber(topicName, Pose, callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # rospy.spin()
        rate.sleep()
        drawnow(PlotPosition2D)