#!/usr/bin/env python3
import rclpy
from time import sleep
from math import radians
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from robotx_interfaces.msg import RobotFrame
from sensor_msgs.msg import JointState

from Arm_Lib.Arm_Lib_Mod import Arm_Device

class MoveAll(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._arm_drv = Arm_Device()
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Start sequence
        self._arm_drv.Arm_serial_set_torque(1)
        self._arm_drv.Arm_RGB_set(50, 0, 50)
        self._arm_drv.Arm_Buzzer_On(1)
        sleep(0.5)
        self._arm_drv.Arm_Buzzer_On(1)
        self._arm_drv.Arm_serial_servo_write6_array((radians(0), radians(0),
                                                     radians(0), radians(0),
                                                     radians(0), 0), 20)
        #self.subscriber=self.create_subscription(RobotFrame, '/dummy/robot_frame', self.frame_callback, self._qos_profile)
        #self.publisher=self.create_publisher(RobotFrame, '/mirror/robot_frame', self._qos_profile)
        self.subscriber=self.create_subscription(JointState, '/dummy/robot_frame', self.frame_callback, self._qos_profile)
        #self.target_position = RobotFrame()
        self.target_position = JointState()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def frame_callback(self, msg:RobotFrame):
        print(msg)
        self.target_position = msg
        """self._arm_drv.Arm_serial_servo_write6_array((int(msg.th1), int(msg.th2), 
                                                     int(msg.th3), int(msg.th4), 
                                                     int(msg.th5), int(msg.g1)), 5)"""
    def timer_callback(self):
        self._arm_drv.Arm_serial_servo_write6_array((int(self.target_position.position[0]), int(self.target_position.position[1]), 
                                                     int(self.target_position.position[2]), int(self.target_position.position[3]), 
                                                     int(self.target_position.position[4]), int(self.target_position.position[5])), 5)

        msg = RobotFrame()
        msg.th1 = self._arm_drv.Arm_serial_servo_read(1)
        msg.th2 = self._arm_drv.Arm_serial_servo_read(2)
        msg.th3 = self._arm_drv.Arm_serial_servo_read(3)
        msg.th4 = self._arm_drv.Arm_serial_servo_read(4)
        msg.th5 = self._arm_drv.Arm_serial_servo_read(5)
        msg.g1  = self._arm_drv.Arm_serial_servo_read(6)
        

def init_node(args=None):
    try:
        rclpy.init(args=args)
        move_all_node = MoveAll('read_node')
        rclpy.spin(move_all_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        move_all_node._arm_drv.Arm_serial_set_torque(0)
        move_all_node._arm_drv.Arm_RGB_set(0, 0, 0)
if __name__=="__main__":
    init_node()
