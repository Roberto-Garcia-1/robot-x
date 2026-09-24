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

  def expand_xacro(self, xacro_path):
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    tmp_path = tmp.name

    cmd = ["ros2", "run", "xacro", "xacro", xacro_path]
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
      raise RuntimeError(f"Error al procesar xacro:\n{result.stderr}")

    with open(tmp_path, "w") as f:
      f.write(result.stdout)

    return tmp_path


  def replace_package_paths(self, urdf_path):
    with open(urdf_path, "r") as f:
      content = f.read()

    def repl(match):
      description_path = get_package_share_directory(
        "robotx_description"
      )
      full = match.group(0)
      rel = full.replace("package://", "")
      pkg = rel.split("/")[0]
      subpath = rel[len(pkg) + 1:]
      #abs_path = os.path.join(ros_ws_src, pkg, subpath)
      abs_path = description_path + "/" + subpath
      return abs_path

    content = re.sub(r'package://[^\"]+', repl, content)

    # guardar nuevo archivo
    new_path = urdf_path.replace(".urdf", "_mujoco.urdf")
    with open(new_path, "w") as f:
      f.write(content)

    return new_path
  def run_mujoco(self, urdf_path, mujoco_path, scene_path):
    print(f"[INFO] Cargando modelo: {urdf_path}")

    # Cargar URDF
    model = mujoco.MjModel.from_xml_path(urdf_path)

    # Exportar a MJCF real
    tmp_xml = urdf_path.replace(".urdf", ".xml")
    mujoco.mj_saveLastXML(tmp_xml, model)

    # Leer MJCF
    with open(tmp_xml, "r") as f:
      xml = f.read()

    # Insertar actuadores
    with open(mujoco_path, "r") as f:
      actuator_block = f.read()
    # Escena
    with open(scene_path, "r") as f:
      scene_block = f.read()

    xml = xml.replace("</mujoco>", actuator_block + "\n</mujoco>")
    #xml = xml.replace("</worldbody>", scene_block + "\n</worldbody>")
    # Crear modelo final
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    print("nu:", model.nu)
    self.model = model
    self.data = data

    for i in range(model.nv):
      name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
      print(i, name)
    with mujoco.viewer.launch_passive(model, data) as viewer:
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
      self.viewer = viewer
    
      """while viewer.is_running():
        target = np.deg2rad(45)
        # aplicar control ANTES del step
        if model.nu > 0:
          pass
        mujoco.mj_step(model, data)
        viewer.sync() 
        print("Contactos:")
        for i in range(data.ncon):
          contact = data.contact[i]

          body1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                  model.geom_bodyid[contact.geom1])
          body2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                  model.geom_bodyid[contact.geom2])
          
          print(f"Contacto entre: {body1} <-> {body2}")
          print("Pos:", contact.pos)"""

"""
# Expandir Xacro a URDF
def expand_xacro(xacro_path):
  tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
  tmp_path = tmp.name

  cmd = ["ros2", "run", "xacro", "xacro", xacro_path]
  result = subprocess.run(cmd, capture_output=True, text=True)

  if result.returncode != 0:
    raise RuntimeError(f"Error al procesar xacro:\n{result.stderr}")

  with open(tmp_path, "w") as f:
    f.write(result.stdout)

  return tmp_path


# Reemplazar "package://"
def replace_package_paths(urdf_path):
  with open(urdf_path, "r") as f:
    content = f.read()

  def repl(match):
    description_path = get_package_share_directory(
      "robotx_description"
    )
    full = match.group(0)
    rel = full.replace("package://", "")
    pkg = rel.split("/")[0]
    subpath = rel[len(pkg) + 1:]
    abs_path = os.path.join(ros_ws_src, pkg, subpath)
    abs_path = description_path + "/" + subpath
    return abs_path

  content = re.sub(r'package://[^\"]+', repl, content)

  # guardar nuevo archivo
  new_path = urdf_path.replace(".urdf", "_mujoco.urdf")
  with open(new_path, "w") as f:
    f.write(content)

  return new_path


# Cargar MuJoCo
def run_mujoco(urdf_path, mujoco_path):
  print(f"[INFO] Cargando modelo: {urdf_path}")

  # Cargar URDF
  model = mujoco.MjModel.from_xml_path(urdf_path)

  # Exportar a MJCF real
  tmp_xml = urdf_path.replace(".urdf", ".xml")
  mujoco.mj_saveLastXML(tmp_xml, model)

  # Leer MJCF
  with open(tmp_xml, "r") as f:
    xml = f.read()

  # Insertar actuadores
  with open(mujoco_path, "r") as f:
    actuator_block = f.read()

  xml = xml.replace("</mujoco>", actuator_block + "\n</mujoco>")
  # Crear modelo final
  model = mujoco.MjModel.from_xml_string(xml)
  data = mujoco.MjData(model)
  print("nu:", model.nu)
  for i in range(model.nv):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(i, name)
  with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    while viewer.is_running():
      target = np.deg2rad(45)
      # aplicar control ANTES del step
      if model.nu > 0:
        pass
      mujoco.mj_step(model, data)
      viewer.sync() 
      print("Contactos:")
      for i in range(data.ncon):
        contact = data.contact[i]

        body1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                model.geom_bodyid[contact.geom1])
        body2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                model.geom_bodyid[contact.geom2])
        
        print(f"Contacto entre: {body1} <-> {body2}")
        print("Pos:", contact.pos)
"""


def main(args=None):
  try:
    
    # Ruta del paquete robot_description
    description_path = get_package_share_directory(
      "robotx_description"
    )
    # Rutas de urdf y rviz conf
    urdf_path = description_path + "/urdf/robotx_mujoco.urdf"
    mujoco_config_file = description_path + "/urdf/robotx_mujoco_config.xml"
    mujoco_scene_file = description_path + "/urdf/mujoco_scene.xml"
    # URDF como xacro

    sim = MujocoSim()
    urdf_tmp = sim.expand_xacro(urdf_path)
    urdf_ready = sim.replace_package_paths(urdf_tmp)

    print("dsadsads")

    # Paso 1: expandir
    urdf_tmp = sim.expand_xacro(urdf_path)
    print(f"[INFO] URDF expandido: {urdf_tmp}")

    # Paso 2: corregir rutas
    urdf_ready = sim.replace_package_paths(urdf_tmp)
    print(f"[INFO] URDF listo para MuJoCo: {urdf_ready}")

    # Paso 3: correr simulación
    sim.run_mujoco(urdf_ready, mujoco_config_file, mujoco_scene_file)
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    pass
  
if __name__ == "__main__":
  main()