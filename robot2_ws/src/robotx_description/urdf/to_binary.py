import os
import trimesh

folder = "/home/robousr/Dev2/robot-x/robot2_ws/src/robotx_description/meshes"

for file in os.listdir(folder):
  if file.lower().endswith(".stl"):
    path = os.path.join(folder, file)
    mesh = trimesh.load(path)
    mesh = mesh.simplify_quadric_decimation(50000)
    mesh.export(path) 
    print(f"Convertido: {file}")