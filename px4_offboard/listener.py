#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from scipy.spatial.transform import Rotation as R

class StateSubscriber(Node):
    def __init__(self):
        super().__init__('state_listener')
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.listener_callback,
            10)

    def listener_callback(self, msg: VehicleOdometry):
        # Position (ENU)
        x, y, z = msg.position

        # Orientation (quaternion -> euler)
        q = msg.q  # [w, x, y, z]
        r = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x, y, z, w]
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)

        print(f"Position -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
        print(f"Orientation -> roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}\n")


def main(args=None):
    rclpy.init(args=args)
    state_subscriber = StateSubscriber()
    rclpy.spin(state_subscriber)
    state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
