#!/usr/bin/env python3
import os
import subprocess
import tempfile
import re
import mujoco
import mujoco.viewer
import numpy as np
# Expandir Xacro a URDF
def expand_xacro(xacro_path):
  with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as tmp:
    tmp_path = tmp.name

  cmd = ["ros2", "run", "xacro", "xacro", xacro_path]
  result = subprocess.run(cmd, capture_output=True, text=True)

  if result.returncode != 0:
    raise RuntimeError(f"Error al procesar xacro:\n{result.stderr}")

  with open(tmp_path, "w") as f:
    f.write(result.stdout)

  return tmp_path


# Reemplazar "package://"
def replace_package_paths(urdf_path, ros_ws_src):
  with open(urdf_path, "r") as f:
    content = f.read()

  def repl(match):
    full = match.group(0)
    rel = full.replace("package://", "")
    pkg = rel.split("/")[0]
    subpath = rel[len(pkg) + 1:]
    abs_path = os.path.join(ros_ws_src, pkg, subpath)
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



def main(args=None):
  try:
    # 🔧 Ajusta estas rutas
    xacro_file = os.path.expanduser(
      "~/Dev2/robot-x/robot2_ws/src/robotx_description/urdf/robotx_mujoco.urdf"
    )
    mujoco_file = os.path.expanduser(
      "~/Dev2/robot-x/robot2_ws/src/robotx_description/urdf/robotx_mujoco_config.xml"
    )

    ros_ws_src = os.path.expanduser("~/Dev2/robot-x/robot2_ws/src")

    # Paso 1: expandir
    urdf_tmp = expand_xacro(xacro_file)
    print(f"[INFO] URDF expandido: {urdf_tmp}")

    # Paso 2: corregir rutas
    urdf_ready = replace_package_paths(urdf_tmp, ros_ws_src)
    print(f"[INFO] URDF listo para MuJoCo: {urdf_ready}")

    # Paso 3: correr simulación
    run_mujoco(urdf_ready, mujoco_file)
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    pass
  
if __name__ == "__main__":
  main()