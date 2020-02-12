#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int16


# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator


class ROSDesiredPositionGenerator(object):
    """ROS interface for publishing desired positions."""

    # write code here for desired position trajectory generator
    def __init__(self):
        # [x, y, z] = self.circle2(1500)
        # print("x=", x)
        # print("y=", y)
        # print("z=", z)

        # Publisher
        self.pub_des_pos = rospy.Publisher('/desired_position', TransformStamped, queue_size=10)  # ???
        self.traj = Int16()
        rospy.Subscriber('/desired_traj', Int16, callback=self._traj_callback)
        # return

    ### TRAJECTORIES
    def _traj_callback(self, msg):
        self.traj = msg

    # linear trajectory (lab1)
    def linear(self, size):
        x = np.zeros(size)
        half_size = int(size / 2)
        x[0:half_size] = np.linspace(-1, 1, half_size)
        x[half_size:] = np.linspace(1, -1, half_size)

        y = np.zeros(size)
        y[0:half_size] = np.linspace(2, 0, half_size)
        y[half_size:] = np.linspace(0, 2, half_size)

        z = np.zeros(size)
        z[0:half_size] = np.linspace(1, 2, half_size)
        z[half_size:] = np.linspace(2, 1, half_size)

        yaw = np.zeros(size)
        return x, y, z, yaw

    # linear trajectory (lab2)
    def linear2(self, size):
        x = np.zeros(size)
        half_size = int(size / 2)
        x[0:half_size] = np.linspace(-1, 1, half_size)
        x[half_size:] = np.linspace(1, -1, half_size)

        y = np.zeros(size)
        y[0:half_size] = np.linspace(2, 0, half_size)
        y[half_size:] = np.linspace(0, 2, half_size)

        z = np.zeros(size)
        z[0:half_size] = np.linspace(1, 2, half_size)
        z[half_size:] = np.linspace(2, 1, half_size)

        yaw = np.zeros(size)
        return x, y, z, yaw

    # circular trajectory (lab1)
    def circle(self, size):
        r = 1
        half_size = int(size / 2)
        theta = np.linspace(0, 2 * np.pi, size)
        x = r * np.cos(theta)
        y = r * np.sin(theta)

        yaw = np.zeros(size)

        z = np.zeros(size)
        z[0:half_size] = np.linspace(0.5, 1.5, half_size)
        z[half_size:] = np.linspace(1.5, 0.5, half_size)
        return x, y, z, yaw

    # circular trajectory (lab2)
    def circle2(self, size):
        r = 1
        half_size = int(size / 2)
        theta = np.linspace(0, 2 * np.pi, size)
        x = r * np.cos(theta)
        y = r * np.sin(theta) + 1
        yaw = np.linspace(-np.pi, 2 * np.pi / 2, size)
        z = np.zeros(size)
        z[0:half_size] = np.linspace(1.5, 1.5, half_size)
        z[half_size:] = np.linspace(1.5, 1.5, half_size)

        return x, y, z, yaw

    # publishing trajectory position and yaw data
    def set_des_pos(self, i, x, y, z, yaw):
        # print("x,y,z: ", x, y, z)
        msg = TransformStamped()
        msg.transform.translation.x = x[i]
        msg.transform.translation.y = y[i]
        msg.transform.translation.z = z[i]
        msg.transform.rotation.z = yaw[i]
        self.pub_des_pos.publish(msg)
        # print("i: ", i, "   msg: ", msg)


if __name__ == '__main__':
    rospy.init_node('desired_position')

    DesPosGen = ROSDesiredPositionGenerator()

    # frequency
    freq = 100
    # update rate
    rate = rospy.Rate(freq)
    # time for flying the trajectory in seconds
    time = 15
    while True:
        try:
            # initializing running variable
            if DesPosGen.traj.data == 1:
                print('linear_trajectory')
                i = 0
                while True:
                    (x, y, z, yaw) = DesPosGen.linear2(freq * time)
                    DesPosGen.set_des_pos(i, x, y, z, yaw)
                    i += 1
                    if i >= time * freq:
                        i = 0
                    rate.sleep()
                    if DesPosGen.traj.data == 2 or DesPosGen.traj.data == 0:
                        break
            elif DesPosGen.traj.data == 2:
                print('circle_trajectory')
                i = 0
                while True:
                    (x, y, z, yaw) = DesPosGen.circle2(freq * time)

                    DesPosGen.set_des_pos(i, x, y, z, yaw)

                    i += 1

                    if i >= time * freq:
                        i = 0
                    rate.sleep()
                    if DesPosGen.traj.data == 1 or DesPosGen.traj.data == 0:
                        break
            else:
                msg = TransformStamped()
                msg.transform.translation.x = 0
                msg.transform.translation.y = 0
                msg.transform.translation.z = 1.5
                msg.transform.rotation.z = 0
                DesPosGen.pub_des_pos.publish(msg)
        except KeyboardInterrupt:
            DesPosGen.set_des_pos(i, x, y, z, yaw)
            rospy.spin()
