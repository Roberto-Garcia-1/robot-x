#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class RobotParamSrv(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Declaracion de parametros
        self.declare_parameter('arm.arm_group.joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('arm.arm_group.poses.zero', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('arm.arm_group.poses.home', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('arm.gripper_group.joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('arm.gripper_group.poses.close', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('arm.gripper_group.poses.open', Parameter.Type.DOUBLE_ARRAY)

        arm_joint_names = Parameter(
            'arm.arm_group.joint_names', 
            Parameter.Type.STRING_ARRAY,
            ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        )
        arm_pose_zero = Parameter(
            'arm.arm_group.poses.zero', 
            Parameter.Type.DOUBLE_ARRAY,
            [0.0, 0.0, 0.0, 0.0, 0.0]
        )
        arm_pose_home = Parameter(
            'arm.arm_group.poses.home', 
            Parameter.Type.DOUBLE_ARRAY,
            [0.0, 0.7854, -1.5708, -1.5708, 0.0]
        )
        gripper_name = Parameter(
            'arm.gripper_group.joint_names', 
            Parameter.Type.STRING_ARRAY,
            ['gripper_joint']
        )
        gripper_pose_close = Parameter(
            'arm.gripper_group.poses.close', 
            Parameter.Type.DOUBLE_ARRAY,
            [0.0]
        )
        gripper_pose_open = Parameter(
            'arm.gripper_group.poses.open', 
            Parameter.Type.DOUBLE_ARRAY,
            [-1.54]
        )

        self.set_parameters(
            [
                arm_joint_names, 
                arm_pose_zero, 
                arm_pose_home, 
                gripper_name, 
                gripper_pose_close, 
                gripper_pose_open]
        )

        self.get_logger().info(f"'{node_name}' parameter server initialized.")


def init_node(args=None):
    try:
        rclpy.init(args=args)
        arm_param_srv_node = RobotParamSrv('arm_param_srv')
        rclpy.spin(arm_param_srv_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Program finalized.')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    init_node()