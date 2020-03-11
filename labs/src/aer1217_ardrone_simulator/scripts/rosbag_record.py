#!/usr/bin/env python2

from __future__ import division, print_function, absolute_import
import rosbag
import rospy
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped


class RecordRosbag(object):
    def __init__(self):
        self._bottom_image_msg = Image()
        self._vicon_msg = TransformStamped()
        rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', TransformStamped, callback=self._vicon_callback)
        rospy.Subscriber('/ardrone/bottom/image_raw', Image, callback=self._bottom_image_callback)

    def _bottom_image_callback(self, msg):
        self._bottom_image_msg = msg

    def _vicon_callback(self, msg):
        self._vicon_msg = msg
