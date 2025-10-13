#!/usr/bin/env python3
import sqlite3
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
import cv2
from datetime import datetime
bridge = CvBridge()

# --- Open rosbag ---
bags_root_path=os.path.expanduser("~/ROS2Dev/rosbags")
# --- Crear base SQLite de salida ---
db_path = os.path.expanduser("~/ROS2Dev/ros2bag/mis_datos.db")
conn = sqlite3.connect(db_path)
# Carpeta para imagenes
os.makedirs("imagenes", exist_ok=True)
cursor = conn.cursor()
cursor.execute("CREATE TABLE IF NOT EXISTS floats (t REAL, d1 REAL, d2 REAL, d3 REAL, d4 REAL, d5 REAL, d6 REAL)")
cursor.execute("CREATE TABLE IF NOT EXISTS imagenes (t_ns INTEGER, timestamp_text TEXT, data BLOB)")
img_count = 0

for subfolder in os.listdir(bags_root_path):
  bag_path = os.path.join(bags_root_path, subfolder)
  if not os.path.isdir(bag_path):
    continue  # Saltar archivos que no sean carpetas
  print(f"Procesando rosbag en: {bag_path}")
  # Preparar reader
  try:
    storage_options = rosbag2_py.StorageOptions(
      uri=bag_path,
      storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
  except Exception as e:
    print(f"Error abriendo rosbag en {bag_path}: {e}")
    continue
  
  # --- Leer mensajes ---
  while reader.has_next():
    (topic, data, t) = reader.read_next()
    t_sec = t * 1e-9
    timestamp_text = datetime.fromtimestamp(t_sec).strftime("%Y-%m-%d %H:%M:%S.%f")
    if topic == "/new_array":
      msg = deserialize_message(data, Float32MultiArray)
      print(len(msg.data))
      cursor.execute("INSERT INTO floats VALUES (?, ?, ?, ?, ?, ?, ?)", (t, *msg.data))
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