#!/usr/bin/env python3
import rclpy
from time import sleep
from math import radians
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from robotx_interfaces.msg import RobotFrame
from math import degrees
from Arm_Lib.Arm_Lib_Mod import Arm_Device
from Arm_Lib.RobotModel import RobotModel
from simple_pid import PID

class MoveAll(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._arm_drv = Arm_Device()
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # PID
        self.current_position = RobotFrame()
        self.target_position = RobotFrame()
        self.target_position_array = [0, 0, 0, 0, 0, 0]
        self.current_position_array = [0, 0, 0, 0, 0, 0]
        self.current_error = [0, 0, 0, 0, 0, 0] 
        self.last_error    = [0, 0, 0, 0, 0, 0]
        self.send_position = [0, 0, 0, 0, 0, 0] 
        self.delta_position = [0, 0, 0, 0, 0, 0] 
        self.kp = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.ki = [0.005, 0.005, 0.005, 0.005, 0.005, 0.005]
        self.kd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.pid = [0, 0, 0, 0, 0, 0]
        self.offset = [0.009995976625058578, 0.14422766273298593, -0.07425582635757677, 0.039983906500233646, 0.1040889132213485, 0.0]

        for i in range(len(self.pid)):
            self.pid[i] = PID(self.kp[i], self.ki[i], self.kd[i], self.target_position_array[i], 0.1)
        # IK
        self.robot_model = RobotModel()
        # Start sequence
        self._arm_drv.Arm_serial_set_torque(1)
        self._arm_drv.Arm_RGB_set(50, 0, 50)
        #self._arm_drv.Arm_Buzzer_On(1)
        sleep(0.5)
        #self._arm_drv.Arm_Buzzer_On(1)
        """self._arm_drv.Arm_serial_servo_write6_array((radians(0), radians(0),
                                                     radians(0), radians(0),
                                                     radians(0), 0), 100)"""
        
        self.publisher=self.create_publisher(RobotFrame, '/mirror/robot_frame', self._qos_profile)
        self.subscriber=self.create_subscription(RobotFrame, '/robot_frame', self.frame_callback, self._qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def frame_callback(self, msg:RobotFrame):
        self.target_position = msg
        self.target_position_array = [self.target_position.th1, self.target_position.th2, self.target_position.th3, 
                                      self.target_position.th4, self.target_position.th5, self.target_position.g1]
        for i in range(len(self.current_error)):
            self.pid[i].setpoint = self.target_position_array[i]
        #self._arm_drv.Arm_serial_servo_write6_array((msg.th1, msg.th2, 
        #                                             msg.th3, msg.th4, 
        #                                             msg.th5, msg.g1), 100)
    def timer_callback(self):
        msg = RobotFrame()
        msg.th1 = self._arm_drv.Arm_serial_servo_read(1)
        msg.th2 = self._arm_drv.Arm_serial_servo_read(2)
        msg.th3 = self._arm_drv.Arm_serial_servo_read(3)
        msg.th4 = self._arm_drv.Arm_serial_servo_read(4)
        msg.th5 = self._arm_drv.Arm_serial_servo_read(5)
        msg.g1  = self._arm_drv.Arm_serial_servo_read(6)
        self.current_position = msg
        self.current_position_array = [self.current_position.th1, self.current_position.th2, self.current_position.th3, 
                                       self.current_position.th4, self.current_position.th5, self.current_position.g1]
        self.current_position_array = [a - b for a, b in zip(self.current_position_array, self.offset)]
        self.last_error = self.current_error
        self.current_error = [a - b for a, b in zip(self.target_position_array, self.current_position_array)]
        for i in range(len(self.current_error)):
            # From current position
            self.delta_position[i] = self.pid[i](self.current_position_array[i])
            max_delta = 0.15
            self.delta_position[i] = max(
                -max_delta,
                min(max_delta, self.delta_position[i])
            )
        

        print("Current   : ", self.current_position_array)
        print("Target    : ", self.target_position_array)
        print("Error     : ", self.current_error)
        print("PID OUTPUT: ", self.delta_position)
        self.send_position = [a + b for a, b in zip(self.current_position_array, self.delta_position)]
        print("NEW OUTPUT: ", self.send_position)
        self._arm_drv.Arm_serial_servo_write6_array(self.send_position, 200)
        print("th1: {:.4f}, th2: {:.4f}, th3: {:.4f}, th4: {:.4f}, th5: {:.4f}, g1: {:.4f}".format(msg.th1, msg.th2, msg.th3, msg.th4, msg.th5, msg.g1))
        x, y, z, gam, bet, al = self.robot_model.direct_kinematics(msg.th1, msg.th2, msg.th3, msg.th4)
        print("x: {:.4f}, y: {:.4f}, z: {:.4f}, alpha: {:.4f}, beta: {:.4f}, gamma: {:.4f}".format(x, y, z, degrees(gam), degrees(bet), degrees(al)))
        self.publisher.publish(msg)
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