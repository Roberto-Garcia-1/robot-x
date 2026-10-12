#!/usr/bin/env python3

import subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

DEFAULT_DESCRIPTION_PACKAGE = "robotx_description"
DEFAULT_INPUT_RELATIVE_PATH = "urdf/robotx_position.urdf"
DEFAULT_OUTPUT_RELATIVE_PATH = "urdf/robotx_position_compiled.urdf"

class URDFFile(Node):
  def __init__(self):
    super().__init__("urdf_file_compiler")
    self.declare_parameter("description_package", DEFAULT_DESCRIPTION_PACKAGE)
    self.declare_parameter("input_relative_path", DEFAULT_INPUT_RELATIVE_PATH)
    self.declare_parameter("output_relative_path", DEFAULT_OUTPUT_RELATIVE_PATH)

    self.description_package = (self.get_parameter("description_package").
                                get_parameter_value().string_value)
    self.input_relative_path = (self.get_parameter("input_relative_path").
                                get_parameter_value().string_value)
    self.output_relative_path = (self.get_parameter("output_relative_path")
        .get_parameter_value().string_value)

    # Construcción de rutas
    try:
      description_path = Path(
        get_package_share_directory(
          self.description_package
        )
      )

    except Exception as e:
      raise RuntimeError(
        f"No se encontró el paquete "
        f"'{self.description_package}': {e}"
      ) from e

    self.urdf_path = (description_path / self.input_relative_path)
    self.output_path = (description_path / self.output_relative_path)

    self.get_logger().info(f"Paquete: {self.description_package}")
    self.get_logger().info(f"Archivo original: {self.urdf_path}")
    self.get_logger().info(f"Archivo compilado: {self.output_path}")

  def process_file(self):
    # Verificar que exista el archivo de entrada
    if not self.urdf_path.is_file():
      raise FileNotFoundError(
        f"No se encontró el archivo original: {self.urdf_path}")
    
    # Ejecutar Xacro
    cmd = ["ros2", "run", "xacro", "xacro", str(self.urdf_path), "-o", str(self.output_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
      raise RuntimeError(f"Error al procesar Xacro:\n{result.stderr}")
    self.get_logger().info("URDF compilado correctamente.")

def main(args=None):
  rclpy.init(args=args)
  node = None

  try:
    node = URDFFile()
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
