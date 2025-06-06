#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# __author__ = "Jaeyoung Lim"
# __contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry

from px4_offboard.controller import quaternion_to_euler
# from scipy.spatial.transform import Rotation as R


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # setup message format
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # get vehicle status
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        # get odometry
        self.imu = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.listener_callback,
            qos_profile)

        # publish control mode
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
        

        # Important message sent to px4 in offboard control
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # run the callback function at 50Hz
        # PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal, 
        # by streaming any of the supported MAVLink setpoint messages or the ROS 2 OffboardControlMode message. 
        # Only need the TrajectorySetpoint once.
        # PX4 enables offboard control only after receiving the signal for more than a second, and will regain control if the signal stops.
        # https://docs.px4.io/main/en/flight_modes/offboard.html
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        # get parameter to generate the path
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        
        # parameters are given in the .launch.py file
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def listener_callback(self, msg:VehicleOdometry):   # msg??
        # Position (ENU)
        self.x, self.y, self.z = msg.position

        # Orientation (quaternion -> euler)
        q = msg.q  # [w, x, y, z]
        self.roll, self.pitch, self.yaw = quaternion_to_euler(q[1],q[2],q[3],q[0])
        # r = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x, y, z, w]
        # self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=False)
        # self.phi_data.append(self.roll)
        # self.theta_data.append(self.pitch)
        # self.psi_data.append(self.yaw)

        # print(f"Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}")
        # print(f"Orientation -> roll: {self.roll:.2f}, pitch: {self.pitch:.2f}, yaw: {self.yaw:.2f}\n")
        self.get_logger().info(f"Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}")
        self.get_logger().info(f"Orientation -> roll: {self.roll:.2f}, pitch: {self.pitch:.2f}, yaw: {self.yaw:.2f}\n")


    def cmdloop_callback(self):
        # Publish offboard control modes
        # https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        # Off-board control mode message contains the following
        # The fields are ordered in terms of priority
        # uint64 timestamp		# time since system start (microseconds)
        # bool position
        # bool velocity
        # bool acceleration
        # bool attitude
        # bool body_rate
        # bool thrust_and_torque
        # bool direct_actuator
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # only control the position
        # https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            # format of this message
            # https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html#trajectorysetpoint-uorb-message
            trajectory_msg = TrajectorySetpoint()
            # trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            # trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            # trajectory_msg.position[2] = -self.theta
            # trajectory_msg.position[2] = -self.altitude
            trajectory_msg.position[0] = 0
            trajectory_msg.position[1] = 0
            trajectory_msg.position[2] = -2
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
