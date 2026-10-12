
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # Argumentos de configuración
  defaults = {
    "description_package"   : "robotx_description",
    "input_relative_path"   : "urdf/robotx_position.urdf",
    "compiled_relative_path": "urdf/robotx_position_compiled.urdf",
    "config_relative_path"  : "mujoco_model/robotx_mujoco_config.xml",
    "scene_relative_path"   : "mujoco_model/robotx_mujoco_scene.xml",
    "output_relative_path"  : "mujoco_model/robotx_mujoco.xml",
  }

  launch_arguments = [
    DeclareLaunchArgument(name, default_value=value, description=f"Parámetro: {name}")
    for name, value in defaults.items()
  ]

  # Compilar URDF/Xacro
  urdf_compile = Node(
    package="robotx_file_compiler",
    executable="urdf_file_compiler",
    name="urdf_file_compiler",
    output="screen",
    parameters=[{
      "description_package": LaunchConfiguration("description_package"),
      "input_relative_path": LaunchConfiguration("input_relative_path"),
      "output_relative_path": LaunchConfiguration("compiled_relative_path"),
    }],
  )

  # Transformar el URDF compilado a MuJoCo
  mujoco_compile = Node(
    package="robotx_file_compiler",
    executable="mujoco_file_transformer",
    name="mujoco_file_compiler",
    output="screen",
    parameters=[{
      "description_package": LaunchConfiguration("description_package"),
      "urdf_relative_path": LaunchConfiguration("compiled_relative_path"),
      "config_relative_path": LaunchConfiguration("config_relative_path"),
      "scene_relative_path": LaunchConfiguration("scene_relative_path"),
      "output_relative_path": LaunchConfiguration("output_relative_path"),
    }],
  )

  # Iniciar la transformación después de que termine la compilación
  start_mujoco_compile = RegisterEventHandler(
    OnProcessExit(
      target_action=urdf_compile,
      on_exit=[mujoco_compile],
    )
  )

  return LaunchDescription(
    launch_arguments + [urdf_compile, start_mujoco_compile]
  )
