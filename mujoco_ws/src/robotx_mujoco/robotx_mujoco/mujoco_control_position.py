#!/usr/bin/env python3

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

        self.positions = [

            [0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

            [0.5, 0.2, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

            [-0.5, 0.4, -0.3, 0.2, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ]

        self.index = 0

        self.timer = self.create_timer(
            1.0,
            self.send_position
        )

    def send_position(self):

        msg = Float64MultiArray()

        msg.data = self.positions[self.index]

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Enviando posición {self.index}"
        )

        self.index = (self.index + 1) % len(self.positions)


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