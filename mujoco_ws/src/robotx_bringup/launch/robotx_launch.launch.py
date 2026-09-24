from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():

  pkg_description = get_package_share_directory(
      'robotx_description')

  robot_description = ParameterValue(
    Command(['xacro ', os.path.join(pkg_description,'urdf', 'robotx_position.urdf')]),
    value_type=str)

  mujoco_node = Node(
      package='mujoco_ros2_control',
      executable='ros2_control_node',
      parameters=[
          {'robot_description': robot_description},
          os.path.join(get_package_share_directory('robotx_description'), 'config', 'controllers.yaml')
      ],
      output='screen'
  )
  joint_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "joint_state_broadcaster",
          "--controller-manager",
          "/controller_manager",
          "--controller-manager-timeout",
          "60"
      ]
  )

  arm_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "arm_controller",
          "--controller-manager",
          "/controller_manager",
          "--controller-manager-timeout",
          "60"
      ]
  )
  start_joint_state_broadcaster = RegisterEventHandler(
    OnProcessStart(
      target_action=mujoco_node,
      on_start=[
        joint_state_broadcaster_spawner
      ]
    )
  )
  start_arm_controller = RegisterEventHandler(
    OnProcessStart(
      target_action=joint_state_broadcaster_spawner,
      on_start=[
        arm_controller_spawner
      ]
    )
  )
  
  return LaunchDescription([
    mujoco_node,
    start_joint_state_broadcaster,
    start_arm_controller
    ])