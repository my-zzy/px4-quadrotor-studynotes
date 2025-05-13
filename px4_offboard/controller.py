#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, ActuatorMotors
import numpy as np
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0.0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class HoverController(Node):
    def __init__(self):
        super().__init__('hover_pid_controller')
        self.hover_thrust = 0.6  # initial guess or from MPC_THR_HOVER param

        self.z_pid = PIDController(kp=0.8, ki=0.05, kd=0.3)
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.target_z = 0.0  # hover at z = 0 in NED (so negative means up)
        self.current_z = 0.0

        qos = rclpy.qos.QoSProfile(depth=10)
        self.sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        self.pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos)

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    def odom_callback(self, msg):
        self.current_z = msg.position[2]  # NED: z is negative above ground

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        error = self.target_z - self.current_z
        correction = self.z_pid.compute(error, dt)

        # Apply correction to hover thrust
        thrust = self.hover_thrust + correction
        thrust = np.clip(thrust, 0.0, 1.0)

        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = int(time.time() * 1e6)
        actuator_msg.control = [thrust] * 4 + [float('nan')] * 8

        self.pub.publish(actuator_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
