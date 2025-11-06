#!/usr/bin/env python3
import sqlite3
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Int32
from robotx_interfaces.msg import RobotFrame
from ament_index_python import get_package_share_directory
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
rosbag_file = "rosbag_11-06_00-32-30"
folder_prefix = get_package_share_directory("robotx_ctrl")+"/../../../../tmp/"
bag_path = folder_prefix + rosbag_file
# --- Crear base SQLite de salida ---
db_path = folder_prefix + rosbag_file + ".db"
conn = sqlite3.connect(db_path)
# Carpeta para imagenes
image_folder = folder_prefix + rosbag_file + "_img"
#os.makedirs(image_folder, exist_ok=True)
cursor = conn.cursor()
cursor.execute("CREATE TABLE IF NOT EXISTS data (task INTEGER, try INTEGER, id INTEGER, t REAL, x REAL, y REAL, z REAL, gamma REAL, beta REAL, alpha REAL, gripper REAL, success INTEGER)")
#cursor.execute("CREATE TABLE IF NOT EXISTS imagenes (t_ns INTEGER, timestamp_text TEXT, data BLOB)")
img_count = 0

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,  # carpeta donde está el bag
    storage_id="sqlite3"
)
converter_options = rosbag2_py.ConverterOptions("", "")
reader = rosbag2_py.SequentialReader()
# --- Leer el mensaje con info de acierto o fallo, tarea e intento
reader.open(storage_options, converter_options)
success = True
task = 1
attempt = 1
while reader.has_next():
    (topic, msg, _) = reader.read_next()
    if topic == "/mirror/robot_frame/success":
      success = deserialize_message(msg, Bool).data
    if topic == "/mirror/robot_frame/task":
      task = deserialize_message(msg, Int32).data
    if topic == "/mirror/robot_frame/attempt":
      attempt = deserialize_message(msg, Int32).data
# --- Leer mensajes ---
reader.open(storage_options, converter_options)
count = 1
while reader.has_next():
  (topic, data, t) = reader.read_next()
  t_sec = t * 1e-9
  timestamp_text = datetime.fromtimestamp(t_sec).strftime("%Y-%m-%d %H:%M:%S.%f")
  if topic == "/mirror/robot_frame":
    msg:RobotFrame = deserialize_message(data, RobotFrame)
    # Cinemática directa
    print("creating data: " + str(t))
    print(msg)
    x, y, z, gam, bet, al = robot_model.direct_kinematics(msg.th1, msg.th2, msg.th3, msg.th4)
    cursor.execute("INSERT INTO data VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", (task, attempt, count, count * 0.1, x, y, z, gam, bet, al, msg.g1, success))
    count += 1
  elif topic == "/camera/image_raw":
    print("image found")
    msg = deserialize_message(data, Image)

    # Convertir a imagen OpenCV con cv_bridge
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Redimensionar a 640x480
    resized = cv2.resize(cv_image, (640, 480))

    # Guardar como archivo JPG
    filename = image_folder + f"/img_{img_count:05d}.jpg"
    cv2.imwrite(filename, resized)
    with open(filename, "rb") as f:
      img_blob = f.read()
      cursor.execute("INSERT INTO imagenes VALUES (?, ?, ?)", (t, timestamp_text, img_blob))




conn.commit()
conn.close()