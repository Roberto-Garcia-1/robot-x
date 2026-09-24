#!/usr/bin/env python3
import os
import subprocess
import tempfile
import re
import mujoco
import mujoco.viewer
import numpy as np
from ament_index_python import get_package_share_directory

class MujocoFile():
  def __init__(self, urdf_path, config_path, scene_path, output_path):
    self.urdf_path = urdf_path
    self.config_path = config_path
    self.scene_path = scene_path
    self.output_path = output_path
    return
  def process_file(self):
    cmd = ["ros2", "run", "xacro", "xacro", self.urdf_path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
      raise RuntimeError(f"Error al procesar xacro:\n{result.stderr}")
    self.xacro_file = result.stdout
    def repl(match):
      description_path = get_package_share_directory("robotx_description")
      full = match.group(0)
      rel = full.replace("package://", "")
      pkg = rel.split("/")[0]
      subpath = rel[len(pkg) + 1:]
      abs_path = description_path + "/" + subpath
      return abs_path

    self.xacro_file = re.sub(r'package://[^\"]+', repl, self.xacro_file)
    self.model = mujoco.MjModel.from_xml_string(self.xacro_file)
    for i in range(self.model.njnt):
      print(
        self.model.joint(i).name,
        self.model.jnt_range[i]
      )
    with tempfile.NamedTemporaryFile(suffix=".xml", delete=False) as f:
      temp_path = f.name
      mujoco.mj_saveLastXML(temp_path, self.model)
    with open(temp_path, "r") as f:
      self.mujoco_file = f.read()
    # Mujoco file print
    # print(self.mujoco_file)
    # self.mujoco_file = self.model.get_xml()

    for line in self.mujoco_file.splitlines():
      if "range=" in line:
        print(line)
    # Configuraciones
    with open(self.config_path, "r") as f:
      config_block = f.read()
    # Escena
    with open(self.scene_path, "r") as f:
      scene_block = f.read()
    self.mujoco_file = self.mujoco_file.replace("</mujoco>",    config_block  + "\n</mujoco>")
    self.mujoco_file = self.mujoco_file.replace("</worldbody>", scene_block   + "\n</worldbody>")
    # Crear modelo final
    print("Modelo final creado. Guardando en {}".format(self.output_path))
    self.model = mujoco.MjModel.from_xml_string(self.mujoco_file)
    for i in range(self.model.njnt):
      print(
        self.model.joint(i).name,
        self.model.jnt_range[i]
      )
    mujoco.mj_saveLastXML(self.output_path, self.model)

def main(args=None):
  try:
    # Ruta del paquete robot_description
    description_path = get_package_share_directory("robotx_description")
    # Rutas de urdf y rviz conf
    urdf_path = description_path + "/urdf/robotx_position.urdf"
    mujoco_config_file = description_path + "/mujoco/robotx_mujoco_config.xml"
    mujoco_scene_file = description_path + "/mujoco/robotx_mujoco_scene.xml"
    mujoco_output_file = description_path + "/mujoco/robotx_mujoco.xml"
    file_tf = MujocoFile(urdf_path, mujoco_config_file, mujoco_scene_file, mujoco_output_file)
    file_tf.process_file()
          
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    pass
  
if __name__ == "__main__":
  main()