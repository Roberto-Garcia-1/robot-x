#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from robotx_interfaces.msg import RobotFrame
from Arm_Lib.Arm_Lib_Mod import Arm_Device
from Arm_Lib.RobotModel import RobotModel
from math import pi, radians, degrees
from time import sleep
from sensor_msgs.msg import JointState

class ReadAll(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._arm_drv = Arm_Device()
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.robot_model = RobotModel()
        #self.publisher = self.create_publisher(RobotFrame, '/dummy/robot_frame', self._qos_profile)
        self.publisher = self.create_publisher(JointState, '/dummy/joint_states', self._qos_profile)
        self.msg = JointState()
        self.msg.name = ["th1", "th2", "th3", "th4", "th5", "g1"]
        # Start sequence
        self._arm_drv.Arm_serial_set_torque(0)
        self._arm_drv.Arm_RGB_set(0, 50, 0)
        self._arm_drv.Arm_Buzzer_On(1)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        #msg = RobotFrame()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        th1 = self._arm_drv.Arm_serial_servo_read(1)
        th2 = self._arm_drv.Arm_serial_servo_read(2)
        th3 = self._arm_drv.Arm_serial_servo_read(3)
        th4 = self._arm_drv.Arm_serial_servo_read(4)
        th5 = self._arm_drv.Arm_serial_servo_read(5)
        g1  = self._arm_drv.Arm_serial_servo_read(6)
        self.msg.position = [th1, th2, th3, th4, th5, g1]
        print("th1: {:.4f}, th2: {:.4f}, th3: {:.4f}, th4: {:.4f}, th5: {:.4f}, g1: {:.4f}".format(th1, th2, th3, th4, th5, g1))
        self.publisher.publish(self.msg)
        """msg.th1 = self._arm_drv.Arm_serial_servo_read(1)
        msg.th2 = self._arm_drv.Arm_serial_servo_read(2)
        msg.th3 = self._arm_drv.Arm_serial_servo_read(3)
        msg.th4 = self._arm_drv.Arm_serial_servo_read(4)
        msg.th5 = self._arm_drv.Arm_serial_servo_read(5)
        msg.g1  = self._arm_drv.Arm_serial_servo_read(6)"""
        
        # print("th1: {:.4f}, th2: {:.4f}, th3: {:.4f}, th4: {:.4f}, th5: {:.4f}, g1: {:.4f}".format(msg.th1, msg.th2, msg.th3, msg.th4, msg.th5, msg.g1))
        # x, y, z, gam, bet, al = self.robot_model.direct_kinematics(msg.th1, msg.th2, msg.th3, msg.th4)
        # print("x: {:.4f}, y: {:.4f}, z: {:.4f}, alpha: {:.4f}, beta: {:.4f}, gamma: {:.4f}".format(x, y, z, degrees(gam), degrees(bet), degrees(al)))
        #self.publisher.publish(msg)
        
def init_node(args=None):
    try:
        rclpy.init(args=args)
        read_all_node = ReadAll('read_node')
        rclpy.spin(read_all_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        read_all_node._arm_drv.Arm_serial_set_torque(0)
        read_all_node._arm_drv.Arm_RGB_set(0, 0, 0)
if __name__=="__main__":
    init_node()
