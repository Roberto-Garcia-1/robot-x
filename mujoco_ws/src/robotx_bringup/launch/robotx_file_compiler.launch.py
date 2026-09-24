from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
  urdf_compile = Node(
    package='robotx_description',
    executable='urdf_file_compiler',
    output='screen'
  )
  mujoco_compile = Node(
    package='robotx_mujoco',
    executable='mujoco_file_transformer',
    output='screen'
  )
  start_mujoco_compile = RegisterEventHandler(
    OnProcessStart(
      target_action=urdf_compile,
      on_start=[mujoco_compile]
    )
  )
  launch_description = LaunchDescription([
    urdf_compile,
    start_mujoco_compile
  ])
  return launch_description