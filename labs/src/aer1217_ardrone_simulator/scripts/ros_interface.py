#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

import os
# Import ROS libraries
import roslib
import rospy
import numpy as np
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist

import matplotlib.pyplot as plt


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""

    # write code here to define node publishers and subscribers
    # publish to /cmd_vel topic
    # subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and attitude feedback

    def __init__(self):
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.pub_vel = rospy.Publisher('/cmd_vel_RHC', Twist, queue_size=32)
        self.vicon_topic = '/vicon/ARDroneCarre/ARDroneCarre'
        self.bottom_image_topic = '/ardrone/bottom/image_raw'
        self._vicon_msg = TransformStamped()
        self._bottom_image_msg = Image()
        rospy.Subscriber(self.vicon_topic, TransformStamped, callback=self._vicon_callback)
        rospy.Subscriber(self.bottom_image_topic, Image, callback=self._bottom_image_callback)

    def land(self):
        self.pub_land.publish()

    def takeoff(self):
        self.pub_takeoff.publish()

    def set_vel(self, roll, pitch, z_dot, yaw_dot):
        msg = Twist()
        msg.linear.x = roll
        msg.linear.y = pitch
        msg.linear.z = z_dot
        msg.angular.z = yaw_dot
        self.pub_vel.publish(msg)

    def _vicon_callback(self, msg):
        self._vicon_msg = msg

    def get_height(self):
        return self._vicon_msg.transform.translation.z

    def get_x(self):
        return self._vicon_msg.transform.translation.x

    def get_y(self):
        return self._vicon_msg.transform.translation.y

    def get_angel_quaternion(self):
        return np.array([self._vicon_msg.transform.rotation.x,
                         self._vicon_msg.transform.rotation.y,
                         self._vicon_msg.transform.rotation.z,
                         self._vicon_msg.transform.rotation.w])

    def get_time(self):
        return rospy.get_time()

    def _bottom_image_callback(self, msg):
        self._bottom_image_msg = msg

    def get_bottom_image(self):
        return self._bottom_image_msg.data

    def get_bottom_image_height(self):
        return self._bottom_image_msg.height

    def get_bottom_image_width(self):
        return self._bottom_image_msg.width


if __name__ == '__main__':
    # write code to create ROSControllerNode

    ardrone = ROSControllerNode()
    rospy.init_node('ros_interface', disable_signals=True)
    rate = rospy.Rate(100)

    ardrone.land()
    d = rospy.Duration(5, 0)
    rospy.sleep(d)

    ardrone.takeoff()
    d = rospy.Duration(2, 0)
    rospy.sleep(d)

    try:
        while True:
            os.system('clear')
            print(ardrone.get_x(),'\n',ardrone.get_y(),'\n',ardrone.get_height(),'\n')
            print(euler_from_quaternion(ardrone.get_angel_quaternion()))
            rate.sleep()
    except KeyboardInterrupt:
        ardrone.set_vel(0, 0, 0, 0)
        ardrone.land()
        rospy.spin()