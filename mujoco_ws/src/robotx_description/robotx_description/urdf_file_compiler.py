#!/usr/bin/env python3
import os
import subprocess
import tempfile
import re
import mujoco
import mujoco.viewer
import numpy as np
from ament_index_python import get_package_share_directory

class URDFFile():
  def __init__(self, urdf_path, output_path):
    self.urdf_path = urdf_path
    self.output_path = output_path
    return

  def process_file(self):
    cmd = ["ros2", "run", "xacro", "xacro", self.urdf_path, "-o", self.output_path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
      raise RuntimeError(f"Error al procesar xacro:\n{result.stderr}")
    return

def main(args=None):
  try:
    # Ruta del paquete robot_description
    description_path = get_package_share_directory("robotx_description")
    # Rutas de urdf y rviz conf
    urdf_path = description_path + "/urdf/robotx_position.urdf"
    output_path = description_path + "/urdf/robotx_position_compiled.urdf"
    file_tf = URDFFile(urdf_path, output_path)
    file_tf.process_file()
          
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    pass
  
if __name__ == "__main__":
  main()