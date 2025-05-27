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
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint

# from scipy.spatial.transform import Rotation as R
from px4_offboard.controller import quaternion_to_euler

from px4_offboard.controller import pd_controller
from px4_offboard.traj import *

from collections import deque
import matplotlib.pyplot as plt

def mapp(x):
    if x > 0.99:
        return 0.99
    elif x < -0.99:
        return -0.99
    else:
        return x

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

        # self.publisher_actuators = self.create_publisher(
        #     ActuatorMotors,
        #     '/fmu/in/actuator_motors',
        #     qos_profile)
        
        self.thrust_pub = self.create_publisher(
            VehicleThrustSetpoint,
            '/fmu/in/vehicle_thrust_setpoint',
            qos_profile)

        self.torque_pub = self.create_publisher(
            VehicleTorqueSetpoint,
            '/fmu/in/vehicle_torque_setpoint',
            qos_profile)

        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.dt = timer_period
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.t = 0.0

        self.dhat = 0
        self.jifen = 0

        self.x_data = deque([0.0]*3, maxlen=3)
        self.y_data = deque([0.0]*3, maxlen=3)
        self.z_data = deque([0.0]*3, maxlen=3)
        self.xd_data = deque([0.0]*3, maxlen=3)
        self.yd_data = deque([0.0]*3, maxlen=3)
        self.zd_data = deque([0.0]*3, maxlen=3)
        self.phi_data = deque([0.0]*3, maxlen=3)
        self.theta_data = deque([0.0]*3, maxlen=3)
        self.psi_data = deque([0.0]*3, maxlen=3)
        self.phid_data = deque([0.0]*3, maxlen=3)
        self.thetad_data = deque([0.0]*3, maxlen=3)
        self.psid_data = deque([0.0]*3, maxlen=3)

        self.x_draw = []
        self.y_draw = []
        self.z_draw = []
        self.x_dot2_draw = []
        self.y_dot2_draw = []
        self.z_dot2_draw = []
        self.phi_draw = []
        self.the_draw = []
        self.psi_draw = []

        self.u1_draw = []
        self.u2_draw = []
        self.u3_draw = []
        self.u4_draw = []


    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.get_logger().info(f"NAV_STATE: {msg.nav_state}, ARMING_STATE: {msg.arming_state}")
    

    def listener_callback(self, msg:VehicleOdometry):   # msg??
        # Position (ENU)
        self.x, self.y, self.z = msg.position
        self.x_data.append(self.x)
        self.y_data.append(self.y)
        self.z_data.append(self.z)

        # Orientation (quaternion -> euler)
        q = msg.q  # [w, x, y, z]
        # r = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x, y, z, w]
        # self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=False)
        self.roll, self.pitch, self.yaw = quaternion_to_euler(q[1],q[2],q[3],q[0])
        self.phi_data.append(self.roll)
        self.theta_data.append(self.pitch)
        self.psi_data.append(self.yaw)

        # self.get_logger().info(f"Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}")
        # self.get_logger().info(f"Orientation -> roll: {self.roll:.2f}, pitch: {self.pitch:.2f}, yaw: {self.yaw:.2f}\n")


    def cmdloop_callback(self):
        now = int(Clock().now().nanoseconds / 1000)

        # Offboard control mode: use actuator control
        # https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = now
        offboard_msg.thrust_and_torque = True
        self.publisher_offboard_mode.publish(offboard_msg)


        # Only send commands if vehicle is armed and in offboard
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            # follow this trajectory
            xdxd, ydyd, zdzd, ppsid = test1(self.t)
            self.xd_data.append(xdxd)
            self.yd_data.append(ydyd)
            self.zd_data.append(zdzd)
            self.psid_data.append(ppsid)

            # pd control preparation
            pos = [self.x_data, self.y_data, self.z_data]
            att = [self.phi_data, self.theta_data, self.psi_data]
            posd = [self.xd_data, self.yd_data, self.zd_data]
            attd = [self.phid_data, self.thetad_data, self.psid_data]

            self.x_draw.append(self.x_data[-1])
            self.y_draw.append(self.y_data[-1])
            self.z_draw.append(self.z_data[-1])
            self.phi_draw.append(self.phi_data[-1])
            self.the_draw.append(self.theta_data[-1])
            self.psi_draw.append(self.psi_data[-1])

            U1, U2, U3, U4, phid_old, thetad_old, dhat_old, jifen_old = pd_controller(pos, att, posd, attd, self.dhat, self.jifen, self.dt)
            self.dhat = dhat_old
            self.jifen = jifen_old
            
            self.phid_data.append(phid_old)
            self.thetad_data.append(thetad_old)

            # self.x_dot2_draw.append(x_dot2)
            # self.y_dot2_draw.append(y_dot2)


            # --- Thrust ---
            max_thrust = 8.54858*4
            thrust_msg = VehicleThrustSetpoint()
            thrust_msg.timestamp = now
            thrust_msg.xyz[0] = 0.0  # No lateral thrust
            thrust_msg.xyz[1] = 0.0
            thrust_msg.xyz[2] = mapp(U1/max_thrust)  # Negative Z = upward in NED/body

            self.thrust_pub.publish(thrust_msg)

            # --- Torque ---
            max_torque = 8.54858*0.174
            max_psi_torque = 1000
            torque_msg = VehicleTorqueSetpoint()
            torque_msg.timestamp = now
            torque_msg.xyz[0] = mapp(U2/max_torque)  # Roll torque
            torque_msg.xyz[1] = mapp(U3/max_torque)  # Pitch torque
            torque_msg.xyz[2] = mapp(U4/max_psi_torque)  # Yaw torque (positive spin)

            self.torque_pub.publish(torque_msg)

            self.get_logger().info(f"Published thrust: {thrust_msg.xyz} | torque: {torque_msg.xyz}")
            self.get_logger().info(f"xyz: {self.x_data[-1]:.2f}, {self.y_data[-1]:.2f}, {self.z_data[-1]:.2f}")
            self.get_logger().info(f"phi: {self.phi_data[-1]:.2f}, thet: {self.theta_data[-1]:.2f}, psi: {self.psi_data[-1]:.2f}")

            self.u1_draw.append(mapp(U1/max_thrust))
            self.u2_draw.append(mapp(U2/max_torque))
            self.u3_draw.append(mapp(U3/max_torque))
            self.u4_draw.append(mapp(U4/max_psi_torque))

            self.t += self.dt

            # draw curve
            test_time = 1000
            if self.t >= test_time*self.dt:
                # Example: plot x, y, z position over time
                # plt.figure()
                # plt.subplot(6, 1, 1)
                # plt.plot(list(range(len(self.phi_draw))), self.phi_draw)
                # plt.title('Roll over time')
                # plt.subplot(6, 1, 2)
                # plt.plot(list(range(len(self.u2_draw))), self.u2_draw)
                # plt.title('U2 over time')
                # plt.subplot(6, 1, 3)
                # plt.plot(list(range(len(self.the_draw))), self.the_draw)
                # plt.title('Pitch over time')
                # plt.subplot(6, 1, 4)
                # plt.plot(list(range(len(self.u3_draw))), self.u3_draw)
                # plt.title('U3 over time')
                # plt.subplot(6, 1, 5)
                # plt.plot(list(range(len(self.x_draw))), self.x_draw)
                # plt.plot(x := np.linspace(0, test_time, 10), 0.3*self.dt*x)
                # plt.title('x over time')
                # plt.subplot(6, 1, 6)
                # plt.plot(list(range(len(self.u1_draw))), self.u1_draw)
                # plt.title('U1 over time')

                # plt.subplot(6, 1, 5)
                # plt.plot(list(range(len(self.z_draw))), self.z_draw)
                # plt.plot(x := np.linspace(0, test_time, 10), -0.5*0.02*x)
                # plt.title('z over time')
                # plt.subplot(6, 1, 6)
                # plt.plot(list(range(len(self.u1_draw))), self.u1_draw)
                # plt.title('U1 over time')
                # plt.tight_layout()

                plt.figure()
                # plt.subplot(6, 1, 1)
                # plt.plot(list(range(len(self.x_dot2_draw))), self.x_dot2_draw)
                # plt.title('x_dot2 over time')
                # plt.subplot(6, 1, 2)
                # plt.plot(list(range(len(self.y_dot2_draw))), self.y_dot2_draw)
                # plt.title('y_dot2 over time')
                plt.subplot(6, 1, 1)
                plt.plot(list(range(len(self.u2_draw))), self.u2_draw)
                plt.title('U2 over time')
                plt.subplot(6, 1, 2)
                plt.plot(list(range(len(self.u3_draw))), self.u3_draw)
                plt.title('U3 over time')
                plt.subplot(6, 1, 3)
                plt.plot(list(range(len(self.phi_draw))), self.phi_draw)
                plt.plot(x := np.linspace(0, test_time, 1000), 0.1*np.sin(self.dt*x))
                plt.title('Roll over time')
                plt.subplot(6, 1, 4)
                plt.plot(list(range(len(self.the_draw))), self.the_draw)
                # plt.plot(x := np.linspace(0, test_time, 1000), 0.2*np.sin(self.dt*x))
                plt.title('Pitch over time')
                plt.subplot(6, 1, 5)
                plt.plot(list(range(len(self.psi_draw))), self.psi_draw)
                plt.title('Yaw over time')
                plt.subplot(6, 1, 6)
                plt.plot(list(range(len(self.u4_draw))), self.u4_draw)
                plt.title('U4 over time')

                # plt.subplot(6, 1, 5)
                # plt.plot(list(range(len(self.x_draw))), self.x_draw)
                # plt.plot(x := np.linspace(0, test_time, 10), 0.3*self.dt*x)
                # plt.title('x over time')
                # plt.subplot(6, 1, 6)
                # plt.plot(list(range(len(self.y_draw))), self.y_draw)
                # plt.plot(x := np.linspace(0, test_time, 10), 0.2*self.dt*x)
                # plt.title('y over time')

                


                plt.show()

                # Stop the node and shutdown ROS
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
