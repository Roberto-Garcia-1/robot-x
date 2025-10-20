#!/usr/bin/env python3
import sqlite3
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
from robotx_interfaces.msg import RobotFrame

from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
import cv2
from datetime import datetime
bridge = CvBridge()
from Arm_Lib.RobotModel import RobotModel
robot_model = RobotModel()

print("Opening database")
# --- Open rosbag ---
bag_path=os.path.expanduser("~/ROS2Dev/ros2bag/model_database")
# --- Crear base SQLite de salida ---
db_path = os.path.expanduser("~/ROS2Dev/ros2bag/model_database.db")
conn = sqlite3.connect(db_path)
# Carpeta para imagenes
os.makedirs("imagenes", exist_ok=True)
cursor = conn.cursor()
cursor.execute("CREATE TABLE IF NOT EXISTS data (task INTEGER, try INTEGER, id INTEGER, t REAL, x REAL, y REAL, z REAL, gamma REAL, beta REAL, alpha REAL, gripper REAL)")
#cursor.execute("CREATE TABLE IF NOT EXISTS imagenes (t_ns INTEGER, timestamp_text TEXT, data BLOB)")
img_count = 0

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,  # carpeta donde está el bag
    storage_id="sqlite3"
)
converter_options = rosbag2_py.ConverterOptions("", "")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# --- Leer mensajes ---
count = 1
while reader.has_next():
  (topic, data, t) = reader.read_next()
  t_sec = t * 1e-9
  timestamp_text = datetime.fromtimestamp(t_sec).strftime("%Y-%m-%d %H:%M:%S.%f")
  if topic == "/robot_frame":
    msg:RobotFrame = deserialize_message(data, RobotFrame)
    # Cinemática directa
    print("creating data: " + str(t))
    print(msg)
    x, y, z, bet, gam, al = robot_model.direct_kinematics(msg.th1, msg.th2, msg.th3, msg.th4)
    cursor.execute("INSERT INTO data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", (1, 1, count, count * 0.1, x, y, z, bet, gam, al, msg.g1))
    count += 1
  elif topic == "/camera/image_raw":
    print("image found")
    msg = deserialize_message(data, Image)

    # Convertir a imagen OpenCV con cv_bridge
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Redimensionar a 640x480
    resized = cv2.resize(cv_image, (640, 480))

    # Guardar como archivo JPG
    filename = f"imagenes/img_{img_count:05d}.jpg"
    cv2.imwrite(filename, resized)
    with open(filename, "rb") as f:
      img_blob = f.read()
      cursor.execute("INSERT INTO imagenes VALUES (?, ?, ?)", (t, timestamp_text, img_blob))




conn.commit()
conn.close()