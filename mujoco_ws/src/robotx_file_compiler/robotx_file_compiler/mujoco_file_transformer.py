#!/usr/bin/env python3

import os
import re
import subprocess
import tempfile
from pathlib import Path
import mujoco
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

DEFAULT_DESCRIPTION_PACKAGE          = "robotx_description"
DEFAULT_URDF_RELATIVE_PATH    = "urdf/robotx_position.urdf"
DEFAULT_CONFIG_RELATIVE_PATH  = "mujoco_model/robotx_mujoco_config.xml"
DEFAULT_SCENE_RELATIVE_PATH   = "mujoco_model/robotx_mujoco_scene.xml"
DEFAULT_OUTPUT_RELATIVE_PATH  = "mujoco_model/robotx_mujoco.xml"

class MujocoFile(Node):
    def __init__(self):
      super().__init__("mujoco_file_compiler")
      # Declaración de parámetros ROS 2
      self.declare_parameter("description_package", DEFAULT_DESCRIPTION_PACKAGE)
      self.declare_parameter("urdf_relative_path", DEFAULT_URDF_RELATIVE_PATH)
      self.declare_parameter("config_relative_path", DEFAULT_CONFIG_RELATIVE_PATH)
      self.declare_parameter("scene_relative_path", DEFAULT_SCENE_RELATIVE_PATH)
      self.declare_parameter("output_relative_path", DEFAULT_OUTPUT_RELATIVE_PATH)

      # Lectura de parámetros
      self.description_package  = self.get_parameter("description_package").value
      self.urdf_relative_path   = self.get_parameter("urdf_relative_path").value
      self.config_relative_path = self.get_parameter("config_relative_path").value
      self.scene_relative_path  = self.get_parameter("scene_relative_path").value
      self.output_relative_path = self.get_parameter("output_relative_path").value

      # Construcción de rutas
      self.urdf_path    = self.get_package_path(self.description_package, self.urdf_relative_path)
      self.config_path  = self.get_package_path(self.description_package, self.config_relative_path)
      self.scene_path   = self.get_package_path(self.description_package, self.scene_relative_path)
      self.output_path  = self.get_package_path(self.description_package, self.output_relative_path)

      self.get_logger().info(f"URDF: {self.urdf_path}")
      self.get_logger().info(f"Configuración: {self.config_path}")
      self.get_logger().info(f"Escena: {self.scene_path}")
      self.get_logger().info(f"Salida: {self.output_path}")

    def get_package_path(self, package_name, relative_path):
      try:
        package_path = Path(get_package_share_directory(package_name))
      except Exception as e:
        raise RuntimeError(f"No se encontró el paquete '{package_name}': {e}") from e
      return package_path / relative_path

    def process_file(self):
      # Verificar archivos de entrada
      for path in (self.urdf_path, self.config_path, self.scene_path):
        if not path.is_file():
          raise FileNotFoundError(f"No se encontró el archivo: {path}")

      # Procesar URDF/Xacro
      cmd = ["ros2", "run", "xacro", "xacro", str(self.urdf_path)]
      result = subprocess.run(cmd, capture_output=True, text=True)
      if result.returncode != 0:
        raise RuntimeError(f"Error al procesar Xacro:\n{result.stderr}")
      self.xacro_file = result.stdout

      # Resolver rutas package:// utilizando el paquete de cada recurso
      def repl(match):
        resource = match.group(0).replace("package://", "", 1)
        package_name, separator, subpath = resource.partition("/")
        if not package_name or not separator or not subpath:
          raise ValueError(f"Ruta package:// inválida: {match.group(0)}")
        return str(Path(get_package_share_directory(package_name)) / subpath)

      self.xacro_file = re.sub(r'package://[^"\s]+', repl, self.xacro_file)

      # Convertir el XML a un modelo MuJoCo
      self.model = mujoco.MjModel.from_xml_string(self.xacro_file)
      temp_path = None
      try:
        with tempfile.NamedTemporaryFile(suffix=".xml", delete=False) as f:
          temp_path = f.name
        mujoco.mj_saveLastXML(temp_path, self.model)
        with open(temp_path, "r") as f:
          self.mujoco_file = f.read()
      finally:
        if temp_path and os.path.exists(temp_path):
          os.remove(temp_path)

      # Agregar configuraciones
      with open(self.config_path, "r") as f:
        config_block = f.read()

      # Agregar escena
      with open(self.scene_path, "r") as f:
        scene_block = f.read()

      self.mujoco_file = self.mujoco_file.replace(
        "</mujoco>", config_block + "\n</mujoco>"
      )
      self.mujoco_file = self.mujoco_file.replace(
        "</worldbody>", scene_block + "\n</worldbody>"
      )

      # Crear modelo final
      self.output_path.parent.mkdir(parents=True, exist_ok=True)
      self.get_logger().info(f"Generando modelo MuJoCo: {self.output_path}")
      self.model = mujoco.MjModel.from_xml_string(self.mujoco_file)
      mujoco.mj_saveLastXML(str(self.output_path), self.model)
      self.get_logger().info("Modelo MuJoCo generado correctamente.")

def main(args=None):
  rclpy.init(args=args)
  node = None
  try:
    node = MujocoFile()
    node.process_file()
  except KeyboardInterrupt:
    print("\nPrograma interrumpido por el usuario.")
  except Exception as e:
    if node is not None:
      node.get_logger().error(str(e))
    else:
      print(f"Error: {e}")
  finally:
    if node is not None:
      node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()

if __name__ == "__main__":
  main()