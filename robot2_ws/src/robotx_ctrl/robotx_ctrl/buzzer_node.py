#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from robotx_interfaces.msg import Buzzer

from .Arm_Lib.Arm_Lib import Arm_Device

class BuzzerCtrl(Node):
    def __init__(self, node_name,):
        super().__init__(node_name)
        self._arm_drv = Arm_Device()
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._buzzer_sub = self.create_subscription(
            Buzzer, 
            '/buzzer_cmd', 
            self._on_buzzer_cmd, 
            self._qos_profile)

    def _on_buzzer_cmd(self, msg:Buzzer):
        delay = msg.delay
        if delay < 0 or delay > 50:
            self.get_logger().warning(f"Buzzer must be between (0-50), delay = {delay}")
        
        if delay == 0:
            self._arm_drv.Arm_Buzzer_Off()
            self.get_logger().info(f"Buzzer Ctrl delay={delay}... OFF")
        else:    
            self._arm_drv.Arm_Buzzer_On(delay=delay)
            self.get_logger().info(f"Buzzer Ctrl delay={delay}... BEEP!!!")
        
def init_node(args=None):
    rclpy.init(args=args)
    buzzer_node = BuzzerCtrl('buzzer_node')
    rclpy.spin(buzzer_node)
    rclpy.shutdown()

if __name__=="__main__":
    init_node()
