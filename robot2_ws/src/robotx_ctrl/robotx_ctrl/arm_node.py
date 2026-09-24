#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import time

from .Arm_Lib.Arm_Lib import Arm_Device

class ArmCtrlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Arm driver class
        self.__arm_drv = Arm_Device()
        # Configuration variables
        self.__arm_servos_resolution = (900, 3100)
        self.__wrist_servo_resolution = (380, 3700)
        self.__arm_range_deg = 180
        self.__wrist_range_deg = 270
        self.__servo_offset = 90
        self.__arm_range_rads = math.radians(self.__arm_range_deg)
        self.__wrist_range_rads = math.radians(self.__wrist_range_deg)
        self.__arm_resolution = (self.__arm_servos_resolution[1] - self.__arm_servos_resolution[0])
        self.__wrist_resolution = (self.__wrist_servo_resolution[1] - self.__wrist_servo_resolution[0])
        # Operational variables
        # self.__gripper_stall_flag = False
        # self.__arm_stall_flag = False
        self.__arm_servos = [0] * 6
        self.__arm_joints = [0.0] * 6
        # Operational variables
        self.__arm_joint_state_pub = None
        self.__gripper_state_pub = None
        
        self.__init_arm()
        
        self.get_logger().info("ArmCtrlNode successfully initialized.")

    def __calc_servo_pos_deg(self, angle: float, is_wrist_servo: bool = False) -> int:
        """
        Calculate the servo position value for a given angle in degrees.

        Args:
            angle (float): Desired angle in degrees.
            is_wrist_servo (bool): Indicates if calculation's parameter need to be
            adjusted to wrist joint servo.

        Returns:
            int: Calculated servo position, clamped within servo range.

        Notes:
            - if is_wrist_servo is false then uses arm joints parameters
            - if is_wrist_servo is true then uses wrist joint parameters
            - Also, gripper servo uses arm joint parameters
        """
        position = 0
        if is_wrist_servo:
            position = (
                    self.__wrist_resolution * angle / self.__wrist_range_deg
                    + self.__wrist_servo_resolution[0]
            )
            # This is 'clamp -> max(min(valor))'
            position = int(
                min(
                    max(position, self.__wrist_servo_resolution[0]),
                    self.__wrist_servo_resolution[1],
                )
            )
        else:
            position = (
                    self.__arm_resolution * angle / self.__arm_range_deg
                    + self.__arm_servos_resolution[0]
            )
            # This is 'clamp -> max(min(valor))'
            position = int(
                min(
                    max(position, self.__arm_servos_resolution[0]),
                    self.__arm_servos_resolution[1],
                )
            )

        return position

    def __calc_servo_pos_rads(self, rads: float, is_wrist_servo: bool = False) -> int:
        """
        Calculate the servo position value for a given angle in radians.

        Args:
            rads (float): Desired angle in radians.
            is_wrist_servo (bool): Indicates if calculation's parameter need to be
            adjusted to wrist joint servo.

        Returns:
            int: Calculated servo position, clamped within servo range.

        Notes:
            - Group 0, arm joints and gripper
            - Group 1, wrist joint, uses wrist resolution parameters
        """
        position = 0
        if is_wrist_servo:
            position = (
                    self.__wrist_resolution * rads / self.__wrist_range_rads
                    + self.__wrist_servo_resolution[0]
            )
            position = int(
                min(
                    max(position, self.__wrist_servo_resolution[0]),
                    self.__wrist_servo_resolution[1],
                )
            )
        else:
            position = (
                    self.__arm_resolution * rads / self.__arm_range_rads
                    + self.__arm_servos_resolution[0]
            )
            position = int(
                min(
                    max(position, self.__arm_servos_resolution[0]),
                    self.__arm_servos_resolution[1],
                )
            )

        return position

    def __calc_servo_angle(self, servo_pos: int, is_wrist_servo: bool = False, in_radians: bool = True) -> float:

        if is_wrist_servo:
            position = (
                    self.__wrist_range_deg * (self.__wrist_servo_resolution[0] * servo_pos) /
                    self.__wrist_resolution
            )
        else:
            position = (
                    self.__arm_range_deg * (self.__arm_servos_resolution[0] * servo_pos) /
                    self.__arm_resolution
            )

        return position if not in_radians else math.radians(position)
    
    def get_serial_servo_angle(self, servo_id: int, in_radians=True) -> float:
        servo_pos = self.__arm_servos[servo_id-1]

        servo_angle = self.__calc_servo_angle(
            servo_pos=servo_pos,
            is_wrist_servo=(servo_id == 5),
            in_radians=in_radians
        )
        servo_angle = (
            (
                (self.__arm_range_rads - servo_angle)
                if in_radians
                else (self.__arm_range_deg - servo_angle)
            )
            if servo_id in (2, 3, 4)
            else servo_angle
        )

        return servo_angle

    def __update_servo_values(self) -> None:
        for i in range(6):
            self.__arm_servos[i] = self.__arm_drv.Arm_serial_servo_read_any(i+1)
            time.sleep(.01)

    def __update_arm_joint_positions(self) -> None:
        for i in range(6):
            self.__arm_joints[i] = self.get_serial_servo_angle(servo_id=(i+1))

    def __init_arm(self):
        self.__update_servo_values()
        self.__update_arm_joint_positions()

    def __arm_move(self, positions, s_time = 500, in_radians=True) -> bool:
        if len(positions) != len(self.__arm_joints)-1:
            self.get_logger().warning(f"Incompatible lenghts, Arm joints({len(self.__arm_joints)}) != positions({len[positions]}): {positions}.")
            return False
        
        if in_radians:
            # Convert received positions from arm range [-1.57, 1.57] in radians 
            # to servo range [0, 180] in degrees
            for j in range(5): positions[j] = math.degrees(positions[j]) + self.__servo_offset

        for i in range(5):
            servo_id = i + 1
            if id == 5:
                time.sleep(.1)
                self.__arm_drv.Arm_serial_servo_write(servo_id, positions[i], int(s_time * 1.2))
            else :
                self.__arm_drv.Arm_serial_servo_write(servo_id, positions[i], s_time)
            # Timeout required to complete i2c call to the coprocessor
            time.sleep(.01)
        
        # Timeout required to complete the arm movement.
        time.sleep(s_time/1000)

        return True
    
    def __full_arm_move(self, positions, s_time = 500, in_radians=True) -> bool:
        if len(positions) != len(self.__arm_joints)-1:
            self.get_logger().warning(f"Incompatible lenghts, Arm joints({len(self.__arm_joints)}) != positions({len[positions]}): {positions}.")
            return False

        if in_radians:
            # Convert received positions from arm range [-1.57, 1.57] in radians 
            # to servo range [0, 180] in degrees
            for j in range(5): positions[j] = math.degrees(positions[j]) + self.__servo_offset

        # Attach current gripper position
        positions.append(math.degrees(self.__arm_joints[5]))
        self.__arm_drv.Arm_serial_servo_write6_array(joints=positions, time=s_time)

    def __gripper_move(self, position, s_time=500, in_radians=True):
        if in_radians:
            position = math.degrees(position)

        self.__arm_drv.Arm_serial_servo_write(6, position, time=s_time)


def init_node(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        arm_ctrl_node = ArmCtrlNode()
        rclpy.spin(arm_ctrl_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    init_node()