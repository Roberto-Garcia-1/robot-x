#!/usr/bin/env python3
import rclpy
from time import sleep
from math import radians
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from robotx_interfaces.msg import RobotFrame

from .Arm_Lib.Arm_Lib_Mod import Arm_Device

class MoveAll(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._arm_drv = Arm_Device()
        self._arm_drv.Arm_serial_set_torque(1)

        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._arm_drv.Arm_serial_servo_write6_array((radians(90), radians(90),
                                                     radians(90), radians(90),
                                                     radians(90), radians(90)), 1000)
        sleep(100)
        self.subscriber=self.create_subscription(RobotFrame, '/robot_frame', self.frame_callback, 10)

    def frame_callback(self, msg:RobotFrame):
        print(msg.th1)
        self._arm_drv.Arm_serial_servo_write6_array((int(msg.th1), int(msg.th2), 
                                                     int(msg.th3), int(msg.th4), 
                                                     int(msg.th5), int(msg.g1)), 5)
def init_node(args=None):
    try:
        rclpy.init(args=args)
        move_all_node = MoveAll('read_node')
        rclpy.spin(move_all_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
if __name__=="__main__":
    init_node()
