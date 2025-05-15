#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry

from scipy.spatial.transform import Rotation as R


class OffboardControl(Node):

    def __init__(self):
        super().__init__('direct_actuator_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        self.imu = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.listener_callback,
            qos_profile)

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)

        self.publisher_actuators = self.create_publisher(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            qos_profile)

        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.dt = timer_period
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Actuator motor values (example: hover power)
        # check "param show MPC_THR_HOVER" under "make px4"
        # for more details of the model, check PX4-gazebo-models in Tools or github
        # https://github.com/PX4/PX4-gazebo-models/tree/main/models
        self.hover_thrust = 0.7  # normalized (0.0 - 1.0)
        self.t = 0.0

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.get_logger().info(f"NAV_STATE: {msg.nav_state}, ARMING_STATE: {msg.arming_state}")
    

    def listener_callback(self, msg: VehicleOdometry):
        # Position (ENU)
        self.x, self.y, self.z = msg.position

        # Orientation (quaternion -> euler)
        q = msg.q  # [w, x, y, z]
        r = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x, y, z, w]
        self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=False)

        print(f"Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}")
        print(f"Orientation -> roll: {self.roll:.2f}, pitch: {self.pitch:.2f}, yaw: {self.yaw:.2f}\n")

    def desire_trajectory(self, t):
        pass
    
    def controller(self, x, y, z, roll, pitch, yaw):
        pass

    def cmdloop_callback(self):
        now = int(Clock().now().nanoseconds / 1000)

        # Offboard control mode: use actuator control
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = now
        offboard_msg.direct_actuator = True
        self.publisher_offboard_mode.publish(offboard_msg)

        # Only send commands if vehicle is armed and in offboard
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            actuator_msg = ActuatorMotors()
            actuator_msg.timestamp = now

            # Example: all 4 motors set to hover thrust
            # https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html
            actuator_msg.control = [
                self.hover_thrust1,
                self.hover_thrust2,
                self.hover_thrust3,
                self.hover_thrust4,
                0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
            ]

            self.publisher_actuators.publish(actuator_msg)

            self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
