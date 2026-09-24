#!/usr/bin/env python3
import rclpy
from time import sleep
from math import radians
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from std_msgs.msg import String
from math import degrees

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import mujoco
import mujoco.viewer
import numpy as np
from robotx_mujoco_sim.mujoco_lib import MujocoSim
from ament_index_python import get_package_share_directory

class MujocoSimNode(Node):
  def __init__(self, node_name):
    super().__init__(node_name)
    self._qos_profile = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      history=HistoryPolicy.KEEP_LAST,
      depth=1
    )
    # =========================
    # MuJoCo
    # =========================
    # Ruta del paquete robot_description
    description_path = get_package_share_directory("robotx_description")
    # Rutas de urdf y rviz conf
    urdf_path = description_path + "/urdf/robotx_mujoco.urdf"
    mujoco_config_file = description_path + "/urdf/robotx_mujoco_config.xml"
    mujoco_scene_file = description_path + "/urdf/mujoco_scene.xml"
    sim = MujocoSim(urdf_path, mujoco_config_file, mujoco_scene_file)

    # Ruta del paquete robot_description
    description_path = get_package_share_directory("robotx_description")
    # Rutas de urdf y rviz conf
    urdf_path = description_path + "/urdf/robotx_mujoco.urdf"
    mujoco_config_file = description_path + "/urdf/robotx_mujoco_config.xml"
    mujoco_scene_file = description_path + "/urdf/mujoco_scene.xml"
    self.mujocosim = MujocoSim(urdf_path, mujoco_config_file, mujoco_scene_file)
    
    """self.model = mujoco.MjModel.from_xml_path("robot.xml")
    self.data = mujoco.MjData(self.model)

    self.viewer = mujoco.viewer.launch_passive(
      self.model,
      self.data
    )"""

    # =========================
    # ROS2
    # =========================

    self.publisher = self.create_publisher(
      JointState,
      "/joint_states",
      10
    )

    self.subscription = self.create_subscription(
      JointState,
      "/joint_commands",
      self.command_callback,
      10
    )

    # =========================
    # Control targets
    # =========================

    self.ctrl = np.zeros(self.mujocosim.model.nu)

    # =========================
    # Timer simulación
    # =========================

    dt = 0.002  # 500 Hz

    self.timer = self.create_timer(
      dt,
      self.timer_callback
    )

  

  def command_callback(self, msg):
    n = min(len(msg.position), self.mujocosim.model.nu)
    for i in range(n):
      self.ctrl[i] = msg.position[i]

  # ===================================
  # Loop simulación
  # ===================================

  def timer_callback(self):
    # aplicar control
    self.mujocosim.data.ctrl[:] = self.ctrl

    # step simulación
    mujoco.mj_step(self.mujocosim.model, self.mujocosim.data)

    # publicar estados
    msg = JointState()

    msg.header.stamp = self.get_clock().now().to_msg()

    msg.name = [
      mujoco.mj_id2name(
        self.mujocosim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        i
      )
      for i in range(self.mujocosim.model.njnt)
    ]

    msg.position = list(self.mujocosim.data.qpos)
    msg.velocity = list(self.mujocosim.data.qvel)

    self.publisher.publish(msg)

    # actualizar viewer
    self.mujocosim.viewer.sync()


def main(args=None):
  try:
    rclpy.init(args=args)
    mujoco_node = MujocoSimNode('mujoco_sim_node')
    rclpy.spin(mujoco_node)
    rclpy.shutdown()
  except KeyboardInterrupt:
    print("\nProgram interrupted by user")
  finally:
    pass
if __name__=="__main__":
    main()