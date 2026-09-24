#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.node import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from robotx_interfaces.msg import RobotFrame
#from robotx_hardware.arm_lib.arm_lib_mod import ArmDevice
#from robotx_hardware.arm_lib.robot_model import RobotModel
from math import pi, radians, degrees
from time import sleep
from sensor_msgs.msg import JointState

class HardwareNode(Node):
  def __init__(self, node_name):
    super().__init__(node_name)
    self.declare_parameter(name="is_sim", value=1)
    #self._arm_drv = ArmDevice()
    self._qos_profile = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      history=HistoryPolicy.KEEP_LAST,
      depth=1
    )
    # Lectura del hardware
    self.publisher = self.create_publisher(JointState, '/joint_states', self._qos_profile)
    self.js_msg = JointState()
    self.js_msg.name = ["th1", "th2", "th3", "th4", "th5", "g1"]
    self.timer = self.create_timer(0.1, self.read_timer_callback)

    # Enviar comandos al hardware
    self.subscriber=self.create_subscription(RobotFrame, '/robot_frame', self.frame_callback, self._qos_profile)
    self.timer = self.create_timer(0.1, self.timer_callback)

  def read_timer_callback(self):
    self.msg.header.stamp = self.get_clock().now().to_msg()
    if self.get_parameter("is_sim"):
      th1, th2, th3, th4, th5, g1 = (0, 0, 0, 0, 0, 0)
      self.js_msg.position = [th1, th2, th3, th4, th5, g1]
      self.publisher.publish(self.js_msg)
      return 
    th1 = self._arm_drv.Arm_serial_servo_read(1)
    th2 = self._arm_drv.Arm_serial_servo_read(2)
    th3 = self._arm_drv.Arm_serial_servo_read(3)
    th4 = self._arm_drv.Arm_serial_servo_read(4)
    th5 = self._arm_drv.Arm_serial_servo_read(5)
    g1  = self._arm_drv.Arm_serial_servo_read(6)
    self.js_msg.position = [th1, th2, th3, th4, th5, g1]
    #print("th1: {:.4f}, th2: {:.4f}, th3: {:.4f}, th4: {:.4f}, th5: {:.4f}, g1: {:.4f}".format(th1, th2, th3, th4, th5, g1))
    self.publisher.publish(self.js_msg)
      

  def frame_callback(self, msg:RobotFrame):
    self.target_position = msg
    self._arm_drv.Arm_serial_servo_write6_array((msg.th1, msg.th2, 
                                                  msg.th3, msg.th4, 
                                                  msg.th5, msg.g1), 5)
  def timer_callback(self):
    return
    msg = RobotFrame()
    msg.th1 = self._arm_drv.Arm_serial_servo_read(1)
    msg.th2 = self._arm_drv.Arm_serial_servo_read(2)
    msg.th3 = self._arm_drv.Arm_serial_servo_read(3)
    msg.th4 = self._arm_drv.Arm_serial_servo_read(4)
    msg.th5 = self._arm_drv.Arm_serial_servo_read(5)
    msg.g1  = self._arm_drv.Arm_serial_servo_read(6)
    
    print("th1: {:.4f}, th2: {:.4f}, th3: {:.4f}, th4: {:.4f}, th5: {:.4f}, g1: {:.4f}".format(msg.th1, msg.th2, msg.th3, msg.th4, msg.th5, msg.g1))
    x, y, z, gam, bet, al = self.robot_model.direct_kinematics(msg.th1, msg.th2, msg.th3, msg.th4)
    print("x: {:.4f}, y: {:.4f}, z: {:.4f}, alpha: {:.4f}, beta: {:.4f}, gamma: {:.4f}".format(x, y, z, degrees(gam), degrees(bet), degrees(al)))
    self.publisher.publish(msg)
        
def main(args=None):
  try:
    rclpy.init(args=args)
    hardware_node = HardwareNode('hardware_node')
    rclpy.spin(hardware_node)
    rclpy.shutdown()
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    #hardware_node._arm_drv.Arm_serial_set_torque(0)
    #hardware_node._arm_drv.Arm_RGB_set(0, 0, 0)
    pass
if __name__=="__main__":
  main()
