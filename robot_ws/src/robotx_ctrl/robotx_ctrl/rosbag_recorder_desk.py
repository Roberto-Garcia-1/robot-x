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
from pynput import keyboard
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
    
    self.get_logger().info("Nodo iniciado. \n[Espacio]:Iniciar/detener grabación \n[x]:Borrar última carpeta \n[t]:Cambiar tarea actual")
    # ROS2Bag
    self.folder_prefix = os.path.expanduser("~/robotx_rosbag/")
    self.task = 1
    self.attempt = 1
    # Hilos para el teclado
    self.keyboard_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
    self.keyboard_thread.start()

  # --- Callbacks de tópicos ---
  def image_callback(self, msg):
    self.image_msg = msg

  def robot_frame_callback(self, msg):
    self.robot_frame_msg = msg

  # --- Control con teclado ---
  def start_keyboard_listener(self):
    def on_press(key):
      try:
        if key == keyboard.Key.space:
          self.toggle_recording()
        elif hasattr(key, 'char') and key.char == 's':
          self.stop_recording_success()
        elif hasattr(key, 'char') and key.char == 'f':
          self.stop_recording_failure()
        elif hasattr(key, 'char') and key.char == 'x':
          self.delete_last_folder()
        elif hasattr(key, 'char') and key.char == 't':
          self.change_task()
        elif key == keyboard.Key.esc:
          print("Exit")
          sys.exit()
      except AttributeError:
        print(f'Special key pressed: {key}')

    listener = keyboard.Listener(on_press=on_press)
    listener.start()



  # --- Grabación ---
  def start_recording(self):
    if not self.recording:
      timestamp = datetime.now().strftime("%d-%m_%H-%M-%S")
      print(timestamp)
      folder_name = self.folder_prefix + f"task{self.task:02}-{self.attempt:05}" + f"rosbag_{timestamp}"
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
      type='robotx_interfaces/msg/RobotFrame',
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
      self.rename_last_folder('s')
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
      self.rename_last_folder('f')
      self.get_logger().info("Grabación guardada como fallo. Presiona [Espacio] para iniciar otra o [x] para eliminar la última carpeta.")
  
  def toggle_recording(self):
    if not self.recording:
      timestamp = datetime.now().strftime("%d-%m_%H-%M-%S")
      print(timestamp)
      self.attempt, folder_name = self.get_next_attempt()
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
      self.recording = False
      self.writer = None
      shutil.rmtree(self.last_folder, ignore_errors=True)
      self.get_logger().info("Grabación detenida. Presiona [Espacio] para iniciar otra o [x] para eliminar la última carpeta.")

  def get_next_attempt(self):
    attempt = 1
    while True:
      base_name = f"task{self.task:02}-{attempt:05}"
      exists = any(
        name.startswith(base_name)
        for name in os.listdir(self.folder_prefix)
      )
      if not exists:
        return attempt, self.folder_prefix + f"task{self.task:02}-{attempt:05}"
      attempt += 1



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
    if self.recording:
      self.get_logger().info("Opción inválida.")
      return
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
  
  def change_task(self):
    if self.recording:
      self.get_logger().info("No se puede cambiar la tarea durante la grabación.")
      return
    try:
      new_task = input("Ingrese el número de tarea: ")
      if not new_task.isdigit():
        self.get_logger().warn("Entrada inválida.")
        return
      self.task = int(new_task)
      self.get_logger().info(f"Tarea actual cambiada a: {self.task}")
    except Exception as e:
      self.get_logger().error(f"Error al cambiar tarea: {e}")
      
  def rename_last_folder(self, result):
    if not self.last_folder:
      self.get_logger().warn("No hay carpeta para renombrar.")
      return

    if not os.path.exists(self.last_folder):
      self.get_logger().warn("La carpeta no existe.")
      return
    new_folder = self.last_folder + f"-{result}"
    try:
      os.rename(self.last_folder, new_folder)
      self.get_logger().info(f"Carpeta renombrada a: {new_folder}")
      self.last_folder = new_folder  # actualizar referencia
    except Exception as e:
      self.get_logger().error(f"Error al renombrar carpeta: {e}")

def init_node(args=None):
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
  init_node()