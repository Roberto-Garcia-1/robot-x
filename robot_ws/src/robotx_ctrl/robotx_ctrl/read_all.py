#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from robotx_interfaces.msg import RobotFrame

from Arm_Lib.Arm_Lib_Mod import Arm_Device

class ReadAll(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        #self._arm_drv = Arm_Device()
        #self._arm_drv.Arm_serial_set_torque(0)

        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(RobotFrame, '/robot_frame', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = RobotFrame()
        self.publisher.publish(msg)
        msg.th1 = self._arm_drv.Arm_serial_servo_read(1)
        msg.th2 = self._arm_drv.Arm_serial_servo_read(2)
        msg.th3 = self._arm_drv.Arm_serial_servo_read(3)
        msg.th4 = self._arm_drv.Arm_serial_servo_read(4)
        msg.th5 = self._arm_drv.Arm_serial_servo_read(5)
        msg.g1  = self._arm_drv.Arm_serial_servo_read(6)
        print(msg)
        self.publisher.publish(msg)
        
def init_node(args=None):
    try:
        rclpy.init(args=args)
        read_all_node = ReadAll('read_node')
        rclpy.spin(read_all_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
if __name__=="__main__":
    init_node()
