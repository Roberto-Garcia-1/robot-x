#!/usr/bin/env python3
import os
import subprocess
import tempfile
import re
import mujoco
import mujoco.viewer
import numpy as np
from ament_index_python import get_package_share_directory

class MujocoSim():
  def __init__(self, urdf_path, config_path, scene_path):
    self.urdf_path = urdf_path
    self.config_path = config_path
    self.scene_path = scene_path
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
    with tempfile.NamedTemporaryFile(suffix=".xml", delete=False) as f:
      temp_path = f.name
      mujoco.mj_saveLastXML(temp_path, self.model)
    with open(temp_path, "r") as f:
      self.mujoco_file = f.read()
    print(self.mujoco_file)
    # self.mujoco_file = self.model.get_xml()
    # Configuraciones
    with open(self.config_path, "r") as f:
      config_block = f.read()
    # Escena
    with open(self.scene_path, "r") as f:
      scene_block = f.read()
    self.mujoco_file = self.mujoco_file.replace("</mujoco>",    config_block  + "\n</mujoco>")
    self.mujoco_file = self.mujoco_file.replace("</worldbody>", scene_block   + "\n</worldbody>")
    # Crear modelo final
    print(self.mujoco_file)
    self.model = mujoco.MjModel.from_xml_string(self.mujoco_file)
    self.data = mujoco.MjData(self.model)
    
    for i in range(self.model.nv):
      name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
      print(i, name)

  def start_mujoco(self):
    with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
      self.viewer = viewer

  def update_mujoco(self):
    while self.viewer.is_running():
      target = np.deg2rad(45)
      # aplicar control ANTES del step
      if self.model.nu > 0:
        pass
      mujoco.mj_step(self.model, self.data)
      self.viewer.sync() 
      print("Contactos:")
      for i in range(self.data.ncon):
        contact = self.data.contact[i]
        body1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                self.model.geom_bodyid[contact.geom1])
        body2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                self.model.geom_bodyid[contact.geom2])
        
        print(f"Contacto entre: {body1} <-> {body2}")
        print("Pos:", contact.pos)

def main(args=None):
  try:
    # Ruta del paquete robot_description
    description_path = get_package_share_directory("robotx_description")
    # Rutas de urdf y rviz conf
    urdf_path = description_path + "/urdf/robotx_mujoco.urdf"
    mujoco_config_file = description_path + "/urdf/robotx_mujoco_config.xml"
    mujoco_scene_file = description_path + "/urdf/mujoco_scene.xml"
    sim = MujocoSim(urdf_path, mujoco_config_file, mujoco_scene_file)
    sim.process_file()
    sim.start_mujoco()
    sim.update_mujoco()
          
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    pass
  
if __name__ == "__main__":
  main()