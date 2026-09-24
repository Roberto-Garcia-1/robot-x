#!/usr/bin/env python3
#coding: utf-8
import smbus
import time
from math import pi
from time import sleep
from typing import List
# V0.1
OFFSET_1 = 0
OFFSET_2 = 100
OFFSET_3 = -50
OFFSET_4 = 20
OFFSET_5 = 80
OFFSET_6 = 0
class Arm_Device(object):
    """DOFBOT Arm controller"""
    def __init__(self):
        """This instance initiates serial comunication with the robot"""
        print("Starting communication")
        self.addr = 0x15
        self.bus = smbus.SMBus(1)
        # This delay allows the serial comunication to properly start
        sleep(2)

    # Set the bus servo angle interface: id: 1-6 (0 means sending 6 servos) angle: 0-180 Set the angle to which the servo will move.
    def Arm_serial_servo_write(self, id:int, angle:float, time:float)->None:
        """
        Writes an angular position to a joint ID
        Args:
            id (int): Joint ID. Send 0 for "All joints"
            angle (float): Angle for the motor (radians)
            time (float): Time (s) to reach the desired position
        Returns:
            None
        """
        if id == 0:  # This is all servo controls
            self.Arm_serial_servo_write6(angle, angle, angle, angle, angle, angle, time)
        elif id == 2 or id == 3 or id == 4:  # Opposite angle to reality
            angle = pi - angle
            pos = int((3100 - 900) * (angle - 0) / (pi - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write I2C error')
        elif id == 5:
            pos = int((3700 - 380) * (angle - 0) / (pi*(3/2.0) - 0) + 380)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write I2C error')
        else:
            pos = int((3100 - 900) * (angle - 0) / (pi - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write I2C error')

    # Set any bus servo angle interface: id: 1-250 (0 is group transmission) angle: 0-180 means 900 3100 0 - 180
    def Arm_serial_servo_write_any(self, id:int, angle:float, time:float)->None:
        """
        Writes an angular position to a motor ID (any on 0-255)
        Args:
            id (int): Motor ID. Send 0 for "All motors"
            angle (float): Angle for the motor (radians)
            time (float): Time (s) to reach the desired position
        Returns:
            None
        """
        if id != 0:
            pos = int((3100 - 900) * (angle - 0) / (pi - 0) + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x19, [id & 0xff, value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write_any I2C error')
        elif id == 0:  # This is all servo controls
            pos = int((3100 - 900) * (angle - 0) / (pi - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x17, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write_any I2C error')

    # Set the bus servo neutral offset with one click, power on and move to the neutral position, and then send the following function, id: 1-6 (setting), 0 (restore to initial)
    def Arm_serial_servo_write_offset_switch(self, id:int)->None:
        """
        Set the bus servo neutral offset with one click, 
        power on and move to the neutral position, and then send 
        the following function, id: 1-6 (setting), 0 (restore to initial)
        Args:
            id (int): Servo desired offset. 1-6 (setting), 0 (restore to initial)
        Returns:
            None
        """
        try:
            if id > 0 and id < 7:
                self.bus.write_byte_data(self.addr, 0x1c, id)
                time.sleep(0.5)
            elif id == 0:
                self.bus.write_byte_data(self.addr, 0x1c, 0x00)
                time.sleep(0.5)
        except:
            print('Arm_serial_servo_write_offset_switch I2C error')

    # Read the status of the one-click setting bus servo mid-bit offset. 
    #   0 means that the corresponding servo ID cannot be found, 
    #   1 means success, and 2 means failure is out of range.
    def Arm_serial_servo_write_offset_state(self)->int:
        """
        Read the status of the one-click setting bus servo mid-bit offset. 
        Returns:
            (int): Servo status
                0 - The corresponding servo ID cannot be found.
                1 - Success.
                2 - Failure. Is out of range.
        """
        try:
            self.bus.write_byte_data(self.addr, 0x1b, 0x01)
            time.sleep(.001)
            state = self.bus.read_byte_data(self.addr, 0x1b)
            return state
        except:
            print('Arm_serial_servo_write_offset_state I2C error')
        return None

    # Set the bus servo angle interface: array
    def Arm_serial_servo_write6_array(self, joints:List[float], time:int)->None:
        """
        Writes a specific angular position to each joint ID
        Args:
            joints (List[float]): Each joint positions (radians)
            time (float): Time (s) to reach the desired position
        Returns:
            None
        """
        s1, s2, s3, s4, s5, s6 = joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]
        if s1 > pi or s2 > pi or s3 > pi or s4 > pi or s5 > pi*(3/4.0) or s6 > pi:
            print("The parameter input range is not within 0-pi")     # print("参数传入范围不在0-180之内！")
            return
        try:
            pos = int((3100 - 900) * (s1 + pi/2.0) / (pi - 0) + 900) + OFFSET_1
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF
            
            #s2 = pi - s2
            pos = int((3100 - 900) * (s2 + pi/2.0) / (pi - 0) + 900) + OFFSET_2
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            #s3 = pi - s3
            pos = int((3100 - 900) * (s3 + pi/2.0) / (pi - 0) + 900) + OFFSET_3
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            #s4 = pi - s4
            pos = int((3100 - 900) * (s4 + pi/2.0) / (pi - 0) + 900) + OFFSET_4
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * (s5 + pi/2.0) / (pi*(3/2.0) - 0) + 380) + OFFSET_5
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3000 - 1060) * s6 + 1060 + OFFSET_6)  # int((3100 - 900) * (s6 - 0) / (pi - 0) + 900)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('Arm_serial_servo_write6 I2C error')

    # Set the bus servo angle interface: s1~S4 and s6: 0-180, S5: 0~270, time is the running time
    def Arm_serial_servo_write6(self, s1:float, s2:float, s3:float, s4:float, s5:float, s6:float, time:float)->None:
        """
        Writes a specific angular position to each joint ID
        Args:
            s1 (float): Joint 1 position (radians)
            s2 (float): Joint 2 position (radians)
            s3 (float): Joint 3 position (radians)
            s4 (float): Joint 4 position (radians)
            s5 (float): Joint 5 position (radians)
            s6 (float): Joint 6 position (radians)
            time (float): Time (s) to reach the desired position
        Returns:
            None
        """
        """if s1 > pi or s2 > pi or s3 > pi or s4 > pi or s5 > pi*(3/4.0) or s6 > pi:
            print("The parameter input range is not within 0-pi")     # print("参数传入范围不在0-180之内！")
            return"""
        try:
            pos = int((3100 - 900) * (s1 + pi/2.0) / (pi - 0) + 900)
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF
            
            #s2 = pi - s2
            pos = int((3100 - 900) * (s2 + pi/2.0) / (pi - 0) + 900)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            #s3 = pi - s3
            pos = int((3100 - 900) * (s3 + pi/2.0) / (pi - 0) + 900)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            #s4 = pi - s4
            pos = int((3100 - 900) * (s4 + pi/2.0) / (pi - 0) + 900)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * (s5 + pi/2.0) / (pi*(3/2.0) - 0) + 380)
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3000 - 1060) * s6 + 1060 + OFFSET_6) # int((3100 - 900) * (s6 - 0) / (pi - 0) + 900)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('Arm_serial_servo_write6 I2C error')

    # Read the specified servo angle, id: 1-6, return 0-180, read error return None
    def Arm_serial_servo_read(self, id:int)->float:
        """
        Read the specified servo angle
        Args:
            id (int): Joint ID
        Returns:
            (float): Joint position (radians). None if error  
        """
        if id < 1 or id > 6:
            print("id must be 1 - 6")
            return None
        try:
            self.bus.write_byte_data(self.addr, id + 0x30, 0x0)
            time.sleep(0.003)
            pos = self.bus.read_word_data(self.addr, id + 0x30)
        except:
            print('Arm_serial_servo_read I2C error')
            return None
        #if pos == 0:
        #    return None
        pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00)
        # print(pos)
        if id == 5:
            pos = (pi*(3/2.0) - 0) * (pos - 380.0) / (3700 - 380) - pi/2.0
            if pos > pi*(1/2.0):
                return float(pi*(1/2.0))
            if pos < -pi*(1/2.0):
                return float(-pi*(1/2.0))
            #if pos > pi*(3/4.0) or pos < 0:
            #    return None
        elif id == 6: #1060-3000 
            pos = (pos - 1060) / (3000 - 1060)
            #pos = pos + OFFSET_GRIPPER
            if pos > 1.0:
                return 1.0
            elif pos < 0:
                return 0.0
        elif id == 1:
            pos = (float(pi) - 0) * (pos - 900) / (3100 - 900) - pi/2.0  
            if pos > pi*(1/2.0):
                return float(pi*(1/2.0))
            elif pos < -pi*(1/2.0):
                return float(-pi*(1/2.0))
            #if pos > pi or pos < 0:
            #    return None
        elif id == 2 or id == 3 or id == 4:
            pos = (float(pi) - 0) * (pos - 900) / (3100 - 900) - pi/2.0
            #print(pos)
            #pos = pi - pos
            if pos > pi*(1/2.0):
                return float(pi*(1/2.0))
            if pos < -pi*(1/2.0):
                return float(-pi*(1/2.0))
        # print(pos)
        return pos

    # Read the bus servo angle, id: 1-250, return 0-pi
    def Arm_serial_servo_read_any(self, id):
        """
        Read the bus servo angle, id: 1-250, return 0-pi
        Args:
            id (int): Servo ID
        Returns:
            (float): Servo position (radians). None if error
        """
        if id < 1 or id > 250:
            print("id must be 1 - 250")
            return None
        try:
            self.bus.write_byte_data(self.addr, 0x37, id)
            time.sleep(0.003)
            pos = self.bus.read_word_data(self.addr, 0x37)
        except:
            print('Arm_serial_servo_read_any I2C error')
            return None
        # print(pos)
        pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00)
        # print(pos)
        pos = float((pi - 0) * (pos - 900) / (3100 - 900) + 0)
        # print(pos)
        return pos

    # Read the servo status, normal return is 0xda, if no data can be read, return 0x00, other values ​​​​are servo errors.
    def Arm_ping_servo(self, id:int)->int:
        """
        Read the servo status, normal return is 0xda, 
        if no data can be read, return 0x00, other values ​​​​are servo errors.
        Args:
            id (int): Servo ID
        Returns:
            (int): Servo status. 0xda: normal, 0x00: no data, other: error
        """
        data = int(id)
        if data > 0 and data <= 250:
            reg = 0x38
            self.bus.write_byte_data(self.addr, reg, data)
            time.sleep(.003)
            value = self.bus.read_byte_data(self.addr, reg)
            times = 0
            while value == 0 and times < 5:
                self.bus.write_byte_data(self.addr, reg, data)
                time.sleep(.003)
                value = self.bus.read_byte_data(self.addr, reg)
                times += 1
                if times >= 5:
                    return None
            return value
        else:
            return None

    # Read hardware version number
    def Arm_get_hardversion(self):
        """
        Read hardware version number
        Returns:
            (int): Hardware version number
        """
        try:
            self.bus.write_byte_data(self.addr, 0x01, 0x01)
            time.sleep(.001)
            value = self.bus.read_byte_data(self.addr, 0x01)
        except:
            print('Arm_get_hardversion I2C error')
            return None
        version = str(0) + '.' + str(value)
        # print(version)
        return version

    # Torque switch 1: Open torque 0: Close torque (can be turned)
    def Arm_serial_set_torque(self, onoff:int):
        """
        Sets torque switch for all motors
        Args: 
            onoff (int): Torque desired status. 1: Open torque 0: Close torque (can be turned)
        Returns:
            None
        """
        try:
            if onoff == 1:
                self.bus.write_byte_data(self.addr, 0x1A, 0x01)
            else:
                self.bus.write_byte_data(self.addr, 0x1A, 0x00)
        except:
            print('Arm_serial_set_torque I2C error')
        sleep(1)

    # Set the bus servo number
    def Arm_serial_set_id(self, id):
        """
        Set the bus servo number
        Args: 
            id (int): Bus servo number
        Returns:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x18, id & 0xff)
        except:
            print('Arm_serial_set_id I2C error')

    # Set the current product color to 1~6, and the RGB light will turn on corresponding color.
    def Arm_Product_Select(self, index:int)->None:
        """
        Set the current product color to 1~6, 
        and the RGB light will turn on corresponding color.
        Args: 
            index (int): Product selection
        Returns:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x04, index & 0xff)
        except:
            print('Arm_Product_Select I2C error')

    # Set RGB lights to specify colors
    def Arm_RGB_set(self, red:int, green:int, blue:int)->None:
        """
        Set RGB lights to specify colors
        Args: 
            red (int): Red color component (0-255)
            green (int): Green color component (0-255)
            blue (int): Blue color component (0-255)
        Returns:
            None
        """
        try:
            self.bus.write_i2c_block_data(self.addr, 0x02, [red & 0xff, green & 0xff, blue & 0xff])
        except:
            print('Arm_RGB_set I2C error')

    # Set K1 button mode, 0: default mode 1: learning mode
    def Arm_Button_Mode(self, mode:int):
        """
        Set K1 button mode, 0: default mode 1: learning mode
        Args: 
            mode (int): K1 Mode. 0: default mode 1: learning mode
        Returns:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x03, mode & 0xff)
        except:
            print('Arm_Button_Mode I2C error')

    # Restart the driver board
    def Arm_reset(self):
        """
        Restart the driver board
        Args: 
            None
        Returs:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x05, 0x01)
        except:
            print('Arm_reset I2C error')

    # PWD servo control id:1-6 (0 controls all servos) angle: 0-180
    def Arm_PWM_servo_write(self, id:int, angle:int)->None:
        """
        Writes an angular position to a joint ID via PWM value
        Args:
            id (int): Joint ID. Send 0 for "All joints"
            angle (float): Angle for the motor (radians)
        Returns:
            None
        """
        try:
            if id == 0:
                self.bus.write_byte_data(self.addr, 0x57, angle & 0xff)
            else:
                self.bus.write_byte_data(self.addr, 0x50 + id, angle & 0xff)
        except:
            print('Arm_PWM_servo_write I2C error')

    # Clear action group
    def Arm_Clear_Action(self):
        """
        Clears action group
        Args:
            None
        Returns:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x23, 0x01)
        except:
            print('Arm_Clear_Action I2C error')

    # In learning mode, record the current action once
    def Arm_Action_Study(self):
        """
        In learning mode, record the current action once
        Args:
            None
        Returns:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x24, 0x01)
        except:
            print('Arm_Action_Study I2C error')

    # Action group operation mode 0: Stop operation 1: Single operation 2: Cycle operation
    def Arm_Action_Mode(self, mode):
        """
        Action group operation mode 0: Stop operation 1: Single operation 2: Cycle operation
        Args:
            mode (int): Desired mode.
                -0: Stop operation 
                -1: Single operation 
                -2: Cycle operation                
        Returns:
            None
        """
        try:
            self.bus.write_byte_data(self.addr, 0x20, mode & 0xff)
        except:
            print('Arm_Clear_Action I2C error')

    # Read the number of saved action groups
    def Arm_Read_Action_Num(self)->int:
        """
        Read the number of saved action groups
        Args:
            None             
        Returns:
            (int): Number of saved action groups
        """
        try:
            self.bus.write_byte_data(self.addr, 0x22, 0x01)
            time.sleep(.001)
            num = self.bus.read_byte_data(self.addr, 0x22)
            return num
        except:
            print('Arm_Read_Action_Num I2C error')

    # Turn on the buzzer, delay defaults to 0xff, and the buzzer keeps sounding.
    # delay=1~50, after turning on the buzzer, it will automatically turn off after delay*100 milliseconds. The maximum delay time is 5 seconds.
    def Arm_Buzzer_On(self, delay:int=0xff)->None:
        """
        Turn on the buzzer, delay defaults to 0xff, and the buzzer keeps sounding.
        delay=1~50, after turning on the buzzer, it will automatically turn off after delay*100 milliseconds. 
        The maximum delay time is 5 seconds.
        Args:
            delay (int): Buzz time (x100ms). Range 0-50 (0-5s).
        Returns:
            None
        """
        if delay != 0:
            self.bus.write_byte_data(self.addr, 0x06, delay&0xff)

    # Turn off the buzzer
    def Arm_Buzzer_Off(self):
        """
        Turn off the buzzer
        Args:
            None
        Returns:
            None
        """
        self.bus.write_byte_data(self.addr, 0x06, 0x00)
