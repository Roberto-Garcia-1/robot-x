#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class RobotXCommander(Node):

    def __init__(self):

        super().__init__("robotx_commander")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/arm_controller/commands",
            10
        )

        self.timer = self.create_timer(
            0.02,   # 50 Hz
            self.timer_callback
        )

        self.t0 = self.get_clock().now()

    def timer_callback(self):

        t = (
            self.get_clock().now().nanoseconds
            - self.t0.nanoseconds
        ) * 1e-9

        q1 = 0.5 * np.sin(t)
        q2 = 0.5 * np.sin(t + np.pi/2)
        q3 = 0.3 * np.sin(0.5*t)
        q4 = 0.0
        q5 = 0.0

        msg = Float64MultiArray()

        msg.data = [
            q1,
            q2,
            q3,
            q4,
            q5,
            0.0,   # joint5a1
            0.0,   # joint5a2
            0.0,   # joint5a3
            0.0,   # joint5b1
            0.0,   # joint5b2
            0.0    # joint5b3
        ]

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = RobotXCommander()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()