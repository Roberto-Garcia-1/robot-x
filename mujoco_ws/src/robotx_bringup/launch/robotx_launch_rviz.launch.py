from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():
  pkg_path = get_package_share_directory('robotx_description')
  rviz_path = pkg_path + "/rviz/rviz.conf.rviz"
  urdf_path = os.path.join(pkg_path,'urdf', 'robotx_position.urdf')
  urdf_xacro  = Command(["xacro ", urdf_path])
  # Modelo URDF como parámetro
  urdf_param = {"robot_description": ParameterValue(urdf_xacro, value_type=str)}
  #Controller
  controller_path = os.path.join(pkg_path, 'config', 'controllers.yaml')

  mujoco_node = Node(
      package='mujoco_ros2_control',
      executable='ros2_control_node',
      parameters=[urdf_param, controller_path],
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
      on_start=[joint_state_broadcaster_spawner]
    )
  )
  start_arm_controller = RegisterEventHandler(
    OnProcessStart(
      target_action=joint_state_broadcaster_spawner,
      on_start=[arm_controller_spawner]
    )
  )
  # Rviz
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_path]
  )
  # Robot description (publica urdf)
  robot_description_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[urdf_param]
  )
  launch_description = LaunchDescription([
    rviz_node, 
    robot_description_node,
    mujoco_node,
    start_joint_state_broadcaster,
    start_arm_controller
  ])
  return launch_description