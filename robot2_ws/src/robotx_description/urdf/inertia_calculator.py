#!/usr/bin/env python3
import os
import math
import argparse
import numpy as np
import trimesh
from lxml import etree

# ---------- Utilidades de rotación ----------
def rpy_to_matrix(roll, pitch, yaw):
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx,  cx]])
    Ry = np.array([[ cy, 0, sy],
                   [  0, 1,  0],
                   [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx  # yaw-pitch-roll (URDF convention)

def origin_to_transform(origin_elem):
    if origin_elem is None:
        R = np.eye(3)
        t = np.zeros(3)
        return R, t

    xyz = origin_elem.get("xyz", "0 0 0").split()
    rpy = origin_elem.get("rpy", "0 0 0").split()
    t = np.array([float(x) for x in xyz])
    r, p, y = [float(a) for a in rpy]
    R = rpy_to_matrix(r, p, y)
    return R, t

# ---------- Resolver package:// ----------
def resolve_package_uri(uri, ros_ws_src):
    if not uri.startswith("package://"):
        return uri
    rel = uri.replace("package://", "")
    pkg = rel.split("/")[0]
    subpath = rel[len(pkg)+1:]
    pkg_path = os.path.join(ros_ws_src, pkg)
    return os.path.join(pkg_path, subpath)

# ---------- Cargar malla con transformación ----------
def load_mesh_with_transform(filename, R, t):
    mesh = trimesh.load(filename, force='mesh')
    if mesh.is_empty:
        return None
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    mesh.apply_transform(T)
    return mesh

# ---------- Procesar un link ----------
def process_link(link_elem, ros_ws_src, density):
    name = link_elem.get("name")
    meshes = []

    # prioridad: visual → collision
    geom_blocks = link_elem.findall("visual")
    if len(geom_blocks) == 0:
        geom_blocks = link_elem.findall("collision")

    for g in geom_blocks:
        origin_elem = g.find("origin")
        R, t = origin_to_transform(origin_elem)

        mesh_elem = g.find(".//mesh")
        if mesh_elem is None:
            continue

        filename = mesh_elem.get("filename")
        filepath = resolve_package_uri(filename, ros_ws_src)

        if not os.path.exists(filepath):
            print(f"[WARN] No existe: {filepath}")
            continue

        mesh = load_mesh_with_transform(filepath, R, t)
        if mesh is not None:
            meshes.append(mesh)

    if not meshes:
        return None

    # Combinar todas las mallas del link
    combined = trimesh.util.concatenate(meshes)
    combined.density = density

    mass = combined.mass
    com = combined.center_mass
    inertia = combined.moment_inertia

    return {
        "name": name,
        "mass": mass,
        "com": com,
        "inertia": inertia
    }

# ---------- Formatear salida URDF ----------
def format_inertial_block(data):
    name = data["name"]
    m = data["mass"]
    com = data["com"]
    I = data["inertia"]

    return f"""
  <!-- ===== {name} ===== -->
  <xacro:property name="{name}_mass" value="{fmt_sci(m)}"/>
  <xacro:property name="{name}_com" value="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}"/>

  <xacro:property name="{name}_ixx" value="{fmt_sci(I[0,0])}"/>
  <xacro:property name="{name}_ixy" value="{fmt_sci(I[0,1])}"/>
  <xacro:property name="{name}_ixz" value="{fmt_sci(I[0,2])}"/>
  <xacro:property name="{name}_iyy" value="{fmt_sci(I[1,1])}"/>
  <xacro:property name="{name}_iyz" value="{fmt_sci(I[1,2])}"/>
  <xacro:property name="{name}_izz" value="{fmt_sci(I[2,2])}"/>
"""
def fmt_sci(x, min_abs=1e-15, sig=5):
    """
    Formatea en notación científica con mínimo valor absoluto.
    
    x: valor original
    min_abs: valor mínimo permitido (evita ceros)
    sig: cifras significativas
    """
    if abs(x) < min_abs:
        x = min_abs if x >= 0 else -min_abs
    return f"{x:.{sig}e}"


# ---------- MAIN ----------
def main():
    print("Start")
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True, help="Ruta al URDF")
    parser.add_argument("--ros_ws_src", required=True,
                        help="Ruta a ws/src (ej: ~/ros2_ws/src)")
    parser.add_argument("--density", type=float, default=2700.0,
                        help="Densidad (kg/m^3)")
    parser.add_argument("--output", default=None,
                        help="Archivo de salida opcional")

    args = parser.parse_args()

    tree = etree.parse(args.urdf)
    root = tree.getroot()

    results = []

    for link in root.findall("link"):
        data = process_link(link, args.ros_ws_src, args.density)
        if data is not None:
            results.append(data)

    output_text = "\n".join([format_inertial_block(d) for d in results])

    print(output_text)

    if args.output:
        with open(args.output, "w") as f:
            f.write(output_text)

if __name__ == "__main__":
    main()