#!/usr/bin/env python3
import os
import select
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy,  HistoryPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CompressedImage
from robotx_interfaces.msg import RobotFrame 
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from datetime import datetime
import threading
from inputimeout import inputimeout, TimeoutOccurred
from std_msgs.msg import Bool, Int32
from ament_index_python import get_package_share_directory
import time
import shutil
import termios
import tty
class BagRecorder(Node):
  def __init__(self):
    super().__init__('bag_recorder')
    # Suscripciones
    self._qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST, depth=1)
    self.image_msg = None
    self.robot_frame_msg = None
    self.create_subscription(Image, '/camera/image_raw', self.image_callback, self._qos_profile)
    self.create_subscription(RobotFrame, '/mirror/robot_frame', self.robot_frame_callback, self._qos_profile)

    # Estado
    self.recording = False
    self.writer = None
    self.last_folder = None

    self.timer = self.create_timer(0.1, self.record_messages)
    self.timer_key = self.create_timer(0.1, self.key_verif)
    
    self.get_logger().info("Nodo iniciado. [Espacio]=Iniciar/detener grabación, [x]=Borrar última carpeta.")
    # ROS2Bag
    self.folder_prefix = get_package_share_directory("robotx_ctrl")+"/../../../../tmp/"
    self.task = 1
    self.attempt = 1
  # --- Callbacks de tópicos ---
  def image_callback(self, msg):
    self.image_msg = msg

  def robot_frame_callback(self, msg):
    self.robot_frame_msg = msg

  # --- Control con teclado ---
  def key_verif(self):
    key = None
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      key = sys.stdin.read(1)

    if key == " ":
      self.start_recording()
    elif key == 's':
      self.stop_recording_success()
    elif key == 'f':
      self.stop_recording_failure()
    elif key == 'x':
      self.delete_last_folder()
    #self.timer_key.reset()



  # --- Grabación ---
  def start_recording(self):
    if not self.recording:
      timestamp = datetime.now().strftime("%m-%d_%H-%M-%S")
      print(timestamp)
      folder_name = self.folder_prefix + f"rosbag_{timestamp}"
      #os.makedirs(folder_name, exist_ok=True)
      self.last_folder = folder_name
      
      self.writer = SequentialWriter()
      storage_options = StorageOptions(
        uri=folder_name,
        storage_id='sqlite3'
      )
      converter_options = ConverterOptions('', '')
      self.writer.open(storage_options, converter_options)

      topic_info = TopicMetadata(
      name='/camera/image_raw',
      type='sensor_msgs/msg/Image',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      topic_info = TopicMetadata(
      name='/mirror/robot_frame',
      type='RobotFrame/msg/RobotFrame',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      topic_info = TopicMetadata(
      name='/mirror/robot_frame/success',
      type='std_msgs/msg/Bool',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      topic_info = TopicMetadata(
      name='/mirror/robot_frame/task',
      type='std_msgs/msg/Int32',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      topic_info = TopicMetadata(
      name='/mirror/robot_frame/attempt',
      type='std_msgs/msg/Int32',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      self.recording = True
      self.get_logger().info(f"Grabando a 10Hz en carpeta: {folder_name}")
    else:
      self.get_logger().info("Opción inválida.")

  def stop_recording_success(self):
    if not self.recording:
      self.get_logger().info("Opción inválida.")
    else:
      self.recording = False
      timestamp = int(self.get_clock().now().nanoseconds)
      res = Bool()
      res.data = True
      attempt = Int32()
      attempt.data = self.attempt
      self.attempt+=1
      task = Int32()
      task.data = self.task
      self.writer.write(
        '/mirror/robot_frame/success',
        serialize_message(res),
        timestamp
      )
      self.writer.write(
        '/mirror/robot_frame/attempt',
        serialize_message(attempt),
        timestamp
      )
      self.writer.write(
        '/mirror/robot_frame/task',
        serialize_message(task),
        timestamp
      )
      self.writer = None
      self.get_logger().info("Grabación guardada como acierto. Presiona [Espacio] para iniciar otra o [x] para eliminar la última carpeta.")
  
  def stop_recording_failure(self):
    if not self.recording:
      self.get_logger().info("Opción inválida.")
    else:
      self.recording = False
      res = Bool()
      res.data = False
      timestamp = int(self.get_clock().now().nanoseconds)
      res = Bool()
      res.data = False
      attempt = Int32()
      attempt.data = self.attempt
      self.attempt+=1
      task = Int32()
      task.data = self.task
      self.writer.write(
        '/mirror/robot_frame/success',
        serialize_message(res),
        timestamp
      )
      self.writer.write(
        '/mirror/robot_frame/attempt',
        serialize_message(attempt),
        timestamp
      )
      self.writer.write(
        '/mirror/robot_frame/task',
        serialize_message(task),
        timestamp
      )
      self.writer = None
      self.get_logger().info("Grabación guardada como fallo. Presiona [Espacio] para iniciar otra o [x] para eliminar la última carpeta.")
  
  def toggle_recording(self):
    if not self.recording:
      timestamp = datetime.now().strftime("%m-%d_%H-%M-%S")
      print(timestamp)
      folder_name = self.folder_prefix + f"rosbag_{timestamp}"
      #os.makedirs(folder_name, exist_ok=True)
      self.last_folder = folder_name
      
      self.writer = SequentialWriter()
      storage_options = StorageOptions(
        uri=folder_name,
        storage_id='sqlite3'
      )
      converter_options = ConverterOptions('', '')
      self.writer.open(storage_options, converter_options)

      topic_info = TopicMetadata(
      name='/camera/image_raw',
      type='sensor_msgs/msg/Image',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      topic_info = TopicMetadata(
      name='/mirror/robot_frame',
      type='RobotFrame/msg/RobotFrame',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)

      self.recording = True
      self.get_logger().info(f"Grabando a 10Hz en carpeta: {folder_name}")
    else:
      self.recording = False
      self.writer = None
      self.get_logger().info("Grabación detenida. Presiona [Espacio] para iniciar otra o [x] para eliminar la última carpeta.")

  def record_messages(self):
    """Se ejecuta a 10Hz"""
    #Verifica teclado
    if not self.recording or self.writer is None:
      return
    timestamp = int(self.get_clock().now().nanoseconds)

    if self.image_msg is not None:
      self.writer.write(
        '/camera/image_raw',
        serialize_message(self.image_msg),
        timestamp
      )

    if self.robot_frame_msg is not None:
      self.writer.write(
        '/mirror/robot_frame',
        serialize_message(self.robot_frame_msg),
        timestamp
      )

  # --- Eliminar última carpeta ---
  def delete_last_folder(self):
    if not self.last_folder:
      self.get_logger().warn("No hay carpeta anterior para borrar.")
      return

    confirm = input(f"¿Seguro que deseas borrar '{self.last_folder}'? (s/n): ")
    if confirm.lower() == 's':
      shutil.rmtree(self.last_folder, ignore_errors=True)
      self.get_logger().info(f"Carpeta '{self.last_folder}' borrada.")
      self.last_folder = None
    else:
      self.get_logger().info("Operación cancelada.")

def main(args=None):
  rclpy.init(args=args)
  node = BagRecorder()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()