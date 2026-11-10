from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

from moveit_configs_utils import MoveItConfigsBuilder

# ============================================================
# CONFIGURACIÓN PREDETERMINADA
# ============================================================

"""
DEFAULT_DESCRIPTION_PACKAGE = "robotx_description"
DEFAULT_URDF_RELATIVE_PATH = "urdf/robotx_position.urdf"
DEFAULT_RVIZ_RELATIVE_PATH = "rviz/rviz_moveit.conf.rviz"
DEFAULT_ROS2_CONTROLLERS_RELATIVE_PATH = "config/controllers.yaml"

DEFAULT_MOVEIT_PACKAGE = "robotx_moveit_config"
DEFAULT_ROBOT_NAME = "dofbot"
DEFAULT_MOVEIT_CONTROLLERS_RELATIVE_PATH = "config/moveit_controllers.yaml"

DEFAULT_JOINT_STATE_BROADCASTER = "joint_state_broadcaster"
DEFAULT_ARM_CONTROLLER = "arm_controller"
DEFAULT_GRIPPER_CONTROLLER = "gripper_controller"

DEFAULT_MUJOCO_PACKAGE = "mujoco_ros2_control"
"""

DEFAULT_DESCRIPTION_PACKAGE = "so100_description"
DEFAULT_URDF_RELATIVE_PATH = "urdf/so100_position.urdf"
DEFAULT_RVIZ_RELATIVE_PATH = "rviz/rviz_moveit.conf.rviz"
DEFAULT_ROS2_CONTROLLERS_RELATIVE_PATH = "config/controllers.yaml"

DEFAULT_MOVEIT_PACKAGE = "so100_moveit"
DEFAULT_ROBOT_NAME = "so100"
DEFAULT_MOVEIT_CONTROLLERS_RELATIVE_PATH = "config/moveit_controllers.yaml"

DEFAULT_JOINT_STATE_BROADCASTER = "joint_state_broadcaster"
DEFAULT_ARM_CONTROLLER = "arm_controller"
DEFAULT_GRIPPER_CONTROLLER = "gripper_controller"

DEFAULT_MUJOCO_PACKAGE = "mujoco_ros2_control"

def launch_setup(context, *args, **kwargs):
  # Lectura de argumentos
  description_package = LaunchConfiguration("description_package").perform(context)
  urdf_relative_path = LaunchConfiguration("urdf_relative_path").perform(context)
  rviz_relative_path = LaunchConfiguration("rviz_relative_path").perform(context)
  ros2_controllers_relative_path = LaunchConfiguration("ros2_controllers_relative_path").perform(context)

  moveit_package = LaunchConfiguration("moveit_package").perform(context)
  robot_name = LaunchConfiguration("robot_name").perform(context)
  moveit_controllers_relative_path = LaunchConfiguration("moveit_controllers_relative_path").perform(context)

  joint_state_broadcaster = LaunchConfiguration("joint_state_broadcaster").perform(context)
  arm_controller = LaunchConfiguration("arm_controller").perform(context)
  gripper_controller = LaunchConfiguration("gripper_controller").perform(context)

  mujoco_package = LaunchConfiguration("mujoco_package").perform(context)
  # Localización de paquetes
  description_path = get_package_share_directory(description_package)
  moveit_path = get_package_share_directory(moveit_package)

  # Construcción de rutas
  urdf_path = os.path.join(description_path, urdf_relative_path)
  rviz_path = os.path.join(description_path, rviz_relative_path)
  ros2_controllers_path = os.path.join(description_path, ros2_controllers_relative_path)
  moveit_controllers_path = os.path.join(moveit_path, moveit_controllers_relative_path)
  #Controller
  controller_path = os.path.join(description_path, 'config', 'controllers.yaml')


  # Procesar URDF/Xacro
  urdf_xacro  = Command(["xacro ", urdf_path])
  # Modelo URDF como parámetro
  urdf_param = {"robot_description": ParameterValue(urdf_xacro, value_type=str)}
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
  gripper_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "gripper_controller",
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
  start_gripper_controller = RegisterEventHandler(
    OnProcessStart(
      target_action=joint_state_broadcaster_spawner,
      on_start=[gripper_controller_spawner]
    )
  )

  # Robot description (publica urdf)
  robot_description_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[urdf_param]
  )

  #MoveIt
  moveit_config = (
  MoveItConfigsBuilder(
    robot_name=robot_name,
    package_name=moveit_package
  ).to_moveit_configs())
  #MoveIt
  moveit_parameters = moveit_config.to_dict()
  #moveit_parameters["robot_description"] = urdf_param["robot_description"]
  # Controller
  moveit_controllers = os.path.join(
    "robotx_moveit_config",
    "config",
    "moveit_controllers.yaml"
  )

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_parameters, moveit_controllers]
  )
  # Rviz
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_path],
    parameters=[
      moveit_config.robot_description,
      moveit_config.robot_description_semantic,
      moveit_config.robot_description_kinematics]
  )
  return [
    rviz_node, 
    robot_description_node,
    mujoco_node,
    start_joint_state_broadcaster,
    start_arm_controller,
    start_gripper_controller,
    move_group_node
  ]


  
def generate_launch_description():
  defaults = {
      "description_package": DEFAULT_DESCRIPTION_PACKAGE,
      "urdf_relative_path": DEFAULT_URDF_RELATIVE_PATH,
      "rviz_relative_path": DEFAULT_RVIZ_RELATIVE_PATH,
      "ros2_controllers_relative_path": DEFAULT_ROS2_CONTROLLERS_RELATIVE_PATH,
      "moveit_package": DEFAULT_MOVEIT_PACKAGE,
      "robot_name": DEFAULT_ROBOT_NAME,
      "moveit_controllers_relative_path": DEFAULT_MOVEIT_CONTROLLERS_RELATIVE_PATH,
      "joint_state_broadcaster": DEFAULT_JOINT_STATE_BROADCASTER,
      "arm_controller": DEFAULT_ARM_CONTROLLER,
      "gripper_controller": DEFAULT_GRIPPER_CONTROLLER,
      "mujoco_package": DEFAULT_MUJOCO_PACKAGE
  }

  launch_arguments = [
      DeclareLaunchArgument(
          name,
          default_value=value,
          description=f"Parámetro: {name}"
      )
      for name, value in defaults.items()
  ]
  return LaunchDescription(
        launch_arguments + [
            OpaqueFunction(function=launch_setup)
        ])