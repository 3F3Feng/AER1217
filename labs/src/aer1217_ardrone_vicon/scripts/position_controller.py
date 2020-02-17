#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

import os
# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Empty, Int16

# import matplotlib.pyplot as plt
# import matplotlib.axes


def main():
    rospy.init_node('position_controller')
    p_control = PositionController()

    rate = rospy.Rate(100)

    try:
        while True:
            if p_control.controller_msg.data == 1:
                i = 0
                while True:
                    p_control.update_state()
                    p_control.update_desired_position(1)
                    i += 1
                    (roll, pitch, z_dot, yaw_dot, dt) = p_control.get_cmd_vel()

                    if i % 10 == 0:
                        # os.system('clear')
                        print('Position:', '\n', p_control.x, '\n', p_control.y, '\n', p_control.z, '\n',
                              'Target_Position:', '\n', p_control.desired_x, '\n', p_control.desired_y, '\n', p_control.desired_z, '\n',
                              'Command:', '\n', roll, '\n', pitch, '\n', z_dot, '\n', yaw_dot,'\n',
                              'error', '\n',
                              p_control.x-p_control.desired_x, '\n',
                              p_control.y-p_control.desired_y, '\n',
                              p_control.z-p_control.desired_z)
                    p_control.send_cmd(roll, pitch, z_dot, yaw_dot)
                    if p_control.controller_msg.data == 0:
                        break
                    # plot some data
                    # fig = plt.figure()
                    # x_data[i-1] = x
                    # y_data[i-1] = y
                    # z_data[i-1] = z

                    # x_des_data[i-1] = p_control.desired_x
                    # y_des_data[i-1] = p_control.desired_y
                    # z_des_data[i-1] = p_control.desired_z

                    # err_x_data[i-1] = x_data[i-1] - x_des_data[i-1]
                    # err_y_data[i-1] = y_data[i-1] - y_des_data[i-1]
                    # err_z_data[i-1] = z_data[i-1] - z_des_data[i-1]

                    # if i==1000:
                    #    break

                    rate.sleep()

                # fig1, ax1 = plt.subplots()
                # print(z_dot_data)
                # print(np.linspace(0,100,100))
                # p1 = ax1.plot(np.linspace(0,10,1000),x_data, np.linspace(0,10,1000),y_data, np.linspace(0,10,1000),z_data)
                # fig2, ax2 = plt.subplots()
                # p2 = ax2.plot(np.linspace(0,10,1000),x_des_data,np.linspace(0,10,1000),y_des_data,np.linspace(0,10,1000),z_des_data)
                # fig3, ax3 = plt.subplots()
                # p3 = ax3.plot(np.linspace(0,10,1000),err_x_data,np.linspace(0,10,1000),err_y_data,np.linspace(0,10,1000),err_z_data)
                # ax1.set_xlim(0,10)
                # ax2.set_xlim(0,10)
                # ax3.set_xlim(0,10)
                # ax2.set_ylim(-1.5,3.5)
                # ax1.set_xlabel('time in sec')
                # ax2.set_xlabel('time in sec')
                # ax3.set_xlabel('time in sec')
                # ax1.set_ylabel('x/y/z in m')
                # ax2.set_ylabel('x_des/y_des/z_des in m')
                # ax3.set_ylabel('err_x/err_y/err_z in m')
                # ax1.legend(['x','y','height'])
                # ax2.legend(['x_des','y_des','z_des'])
                # ax3.legend(['err_x','err_y','err_z'])
                # plt.show()

    except KeyboardInterrupt:
        p_control.send_cmd(0, 0, 0, 0)
        p_control.pub_land.publish()
        rospy.spin()


class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""

    # write code here for position controller
    def __init__(self):

        # Publisher
        self.pub_cmd = rospy.Publisher('/cmd_vel_RHC', Twist, queue_size=32)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)

        # Subscriber
        self._dp_msg = TransformStamped()
        self._vel_msg = Twist()
        self._vicon_msg = TransformStamped()
        self.controller_msg = Int16()
        rospy.Subscriber('/desired_position', TransformStamped, callback=self._dp_callback)
        rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', TransformStamped, callback=self._vicon_callback)
        rospy.Subscriber('/controller', Int16, callback=self._controller_callback)
        ### initializing variables

        # gravitational acceleration
        self._g = -9.8

        # desired position variables
        self.desired_x = 0
        self.desired_y = 0
        self.desired_z = 0
        self.desired_yaw = 0

        self.desired_x_old = 0
        self.desired_y_old = 0
        self.desired_z_old = 0
        self.desired_yaw_old = 0

        # actual position
        self.x = 0
        self.y = 0
        self.z = 0

        # old position
        self.x_old = 0
        self.y_old = 0
        self.z_old = 0

        # actual angles
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # old angles
        self.roll_old = 0
        self.pitch_old = 0
        self.yaw_old = 0

        # time
        self.cur_t = 0.0
        self.old_t = 0.0

        # derivatives
        self.x_dot = 0
        self.y_dot = 0
        self.z_dot = 0

        ### Offboard controller gains
        # damping parameter
        self.zeta_roll = 1
        # natural frequency
        self.omega_roll = 3.5 / 1.8

        self.zeta_pitch = 1
        self.omega_pitch = 3.5 / 1.8

        self.zeta_climb = 0.9
        self.omega_climb = 3 / 1.8

        self.zeta_yaw = 0.9
        self.omega_yaw = .2 / 1.8
        self.kp_yaw = 20

        # current state
        # self._x_error = 0.0
        # self._y_error = 0.0
        # self._z_error = 0.0

        # self.climb_rate_old = 0.0

    def _dp_callback(self, msg):
        self._dp_msg = msg
        # print("dp_msg: ", self._dp_msg)

    def _vicon_callback(self, msg):
        self._vicon_msg = msg

    def _controller_callback(self, msg):
        self.controller_msg = msg

    # update drone state
    def update_state(self):

        # store old position data
        (self.x_old, self.y_old, self.z_old) = (self.x, self.y, self.z)
        # store old angular data
        (self.roll_old, self.pitch_old, self.yaw_old) = (self.roll, self.pitch, self.yaw)
        # store old time
        self.old_t = self.cur_t

        # update position data
        (self.x,
         self.y,
         self.z) = (self._vicon_msg.transform.translation.x,
                    self._vicon_msg.transform.translation.y,
                    self._vicon_msg.transform.translation.z)

        quaternion = np.array([self._vicon_msg.transform.rotation.x,
                               self._vicon_msg.transform.rotation.y,
                               self._vicon_msg.transform.rotation.z,
                               self._vicon_msg.transform.rotation.w])

        euler = euler_from_quaternion(quaternion)

        # update angular data
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        # print("roll, pitch, yaw: ",self.roll,self.pitch,self.yaw)

        # update time
        self.cur_t = rospy.get_time()

    # update desired position data
    def update_desired_position(self, r):
        # store old desired position data
        (self.desired_x_old, self.desired_y_old, self.desired_z_old, self.desired_yaw_old) = (
            self.desired_x, self.desired_y, self.desired_z, self.desired_yaw)

        # update desired position data
        (self.desired_x, self.desired_y, self.desired_z) = (
            self._dp_msg.transform.translation.x, self._dp_msg.transform.translation.y,
            self._dp_msg.transform.translation.z)
        quat = [self._dp_msg.transform.rotation.x,
                self._dp_msg.transform.rotation.y,
                self._dp_msg.transform.rotation.z,
                self._dp_msg.transform.rotation.w]
        euler = euler_from_quaternion(quat)
        self.desired_yaw = euler[2]
        # print("x,y,z_des: ",self.desired_x, self.desired_y, self.desired_z)

        # if r=0, ignore desired position data and just hover
        if r == 0:
            self.desired_x = 0
            self.desired_y = 0
            self.desired_z = 2
            self.desired_yaw = 0

        return

    # calculate the commands
    def get_cmd_vel(self):

        # calculate time step
        dt = (self.cur_t - self.old_t)

        ### calculating derivatives
        # calculate actual and desired yaw velocity

        while self.desired_yaw > np.pi:
            self.desired_yaw = self.desired_yaw - 2 * np.pi

        while self.desired_yaw < -np.pi:
            self.desired_yaw = self.desired_yaw + 2 * np.pi

        while self.yaw_old > np.pi:
            self.yaw_old = self.yaw_old - 2 * np.pi

        while self.yaw_old < -np.pi:
            self.yaw_old = self.yaw_old + 2 * np.pi

        yaw_dot = (self.yaw - self.yaw_old) / dt
        yaw_err = self.desired_yaw - self.yaw_old

        while yaw_err > np.pi:
            yaw_err = yaw_err - 2 * np.pi

        while yaw_err < -np.pi:
            yaw_err = yaw_err + 2 * np.pi

        desired_yaw_dot = (yaw_err) / dt
        cmd_yaw_rate = desired_yaw_dot/self.kp_yaw

        # actual x velocity
        x_dot = (self.x - self.x_old) / dt
        # actual y velocity
        y_dot = (self.y - self.y_old) / dt
        # actual z velocity
        z_dot = (self.z - self.z_old) / dt

        # desired x velocity
        desired_x_dot = (self.desired_x - self.desired_x_old) / dt
        # desired y velocity
        desired_y_dot = (self.desired_y - self.desired_y_old) / dt
        # desired z velocity
        desired_z_dot = (self.desired_z - self.desired_z_old) / dt

        ### position error calculation
        # error in x position
        err_x = self.desired_x - self.x
        # error in y position
        err_y = self.desired_y - self.y
        # error in z position
        err_z = self.desired_z - self.z

        ### velocity error calculation
        # error in x velocity
        err_x_dot = desired_x_dot - x_dot
        # error in y velocity
        err_y_dot = desired_y_dot - y_dot
        # error in z velocity
        err_z_dot = desired_z_dot - z_dot
        # error in y velocity
        err_yaw_dot = desired_yaw_dot - yaw_dot

        ### second derivatives (using second order equation)
        # commanded x acceleration
        x_2dot = 2 * self.zeta_pitch * self.omega_pitch * err_x_dot + self.omega_pitch ** 2 * err_x
        # commanded y acceleration
        y_2dot = 2 * self.zeta_roll * self.omega_roll * err_y_dot + self.omega_roll ** 2 * err_y
        # commanded z acceleration
        z_2dot = 2 * self.zeta_climb * self.omega_climb * err_z_dot + self.omega_climb ** 2 * err_z

        # calculate mass-normalized motor thrust
        f = (z_2dot + self._g) / (np.cos(self.roll) * np.cos(self.pitch))

        ### calculation of commanded roll and pitch in earth frame
        ## roll

        # arcsin argument calculation
        ro = -y_2dot / f

        # limiting argument
        if ro > 1:
            ro = 1
        elif ro < -1:
            ro = -1

        # commanded roll
        cmd_roll_earth = np.arcsin(ro)

        ## pitch

        # arcsin argument calculation
        pi = x_2dot / (f * np.cos(self.roll))

        # limiting argument
        if pi > 1:
            pi = 1
        elif pi < -1:
            pi = -1

            # commanded pitch
        cmd_pitch_earth = np.arcsin(pi)

        ### transformation of roll and pitch from earth frame to body frame
        cmd_roll_body = cmd_roll_earth * np.cos(self.yaw) + cmd_pitch_earth * np.sin(self.yaw)
        cmd_pitch_body = -cmd_roll_earth * np.sin(self.yaw) + cmd_pitch_earth * np.cos(self.yaw)

        # limiting angular commands
        if cmd_roll_body > 1:
            cmd_roll_body = 1
        elif cmd_roll_body < -1:
            cmd_roll_body = -1

        if cmd_pitch_body > 1:
            cmd_pitch_body = 1
        elif cmd_pitch_body < -1:
            cmd_pitch_body = -1

        return -cmd_roll_body, -cmd_pitch_body, err_z, cmd_yaw_rate, dt

    def send_cmd(self, roll, pitch, z_dot, yaw_dot):
        msg = Twist()
        msg.linear.x = roll
        msg.linear.y = pitch
        msg.linear.z = z_dot
        msg.angular.z = yaw_dot
        self.pub_cmd.publish(msg)


if __name__ == '__main__':
    main()
