#!/usr/bin/env python3
import os
import select
import sys

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CompressedImage
from robotx_interfaces.msg import RobotFrame 
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from datetime import datetime
import threading
from inputimeout import inputimeout, TimeoutOccurred

import time
import shutil
import termios
import tty
class BagRecorder(Node):
  def __init__(self):
    super().__init__('bag_recorder')
    # Suscripciones
    self.image_msg = None
    self.robot_frame_msg = None
    self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
    self.create_subscription(RobotFrame, '/robot_frame', self.robot_frame_callback, 10)

    # Estado
    self.recording = False
    self.writer = None
    self.last_folder = None

    self.timer = self.create_timer(0.1, self.record_messages)
    self.timer_key = self.create_timer(0.1, self.key_verif)
    # Hilos
    """self.keyboard_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
    self.keyboard_thread.start()"""

    self.get_logger().info("Nodo iniciado. [Espacio]=Iniciar/detener grabación, [x]=Borrar última carpeta.")

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
        elif hasattr(key, 'char') and key.char == 'x':
          self.delete_last_folder()
      except AttributeError:
        print(f'Special key pressed: {key}')

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

  
  """def key_verif(self):
    self.timer_key.cancel()
    #key = input("")
    try:
        key = inputimeout(timeout=0.05)
    except TimeoutOccurred:
        key = None
    if key == " ":
      self.toggle_recording()
    elif key == 'x':
      self.delete_last_folder()
    self.timer_key.reset()"""
  def key_verif(self):
    key = None
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      key = sys.stdin.read(1)
    if key == " ":
      self.toggle_recording()
    elif key == 'x':
      self.delete_last_folder()
    #self.timer_key.reset()



  # --- Grabación ---
  def toggle_recording(self):
    if not self.recording:
      timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
      folder_name = f"rosbags/rosbag_{timestamp}"
      #os.makedirs(folder_name, exist_ok=True)
      self.last_folder = folder_name

      
      
      self.writer = SequentialWriter()
      storage_options = StorageOptions(
        uri=folder_name,
        storage_id='sqlite3'
      )
      converter_options = ConverterOptions('', '')
      self.writer.open(storage_options, converter_options)
      
      """self.writer.create_topic({
        'name': '/camera/image_raw',
        'type': 'sensor_msgs/msg/Image',
        'serialization_format': 'cdr'
      })
      self.writer.create_topic({
        'name': '/robot_frame',
        'type': 'your_package_name/msg/RobotFrame',  # cambia aquí
        'serialization_format': 'cdr'
      })"""

      topic_info = TopicMetadata(
      name='/camera/image_raw',
      type='sensor_msgs/msg/Image',
      serialization_format='cdr')
      self.writer.create_topic(topic_info)
      topic_info = TopicMetadata(
      name='/robot_frame',
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
    """key = self.get_key(termios.tcgetattr(sys.stdin))
    if key == " ":
      self.toggle_recording()
    elif key == 'x':
      self.delete_last_folder()"""
    if not self.recording or self.writer is None:
      return
    now = self.get_clock().now().to_msg()
    timestamp = int(self.get_clock().now().nanoseconds)

    if self.image_msg is not None:
      self.writer.write(
        '/camera/image_raw',
        serialize_message(self.image_msg),
        timestamp
      )

    if self.robot_frame_msg is not None:
      self.writer.write(
        '/robot_frame',
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






#!/usr/bin/env python3
import os
import select
import sys

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CompressedImage
from robotx_interfaces.msg import RobotFrame 
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from datetime import datetime
import threading
from pynput import keyboard
import time
import shutil
import termios
import tty

class BagRecorder(Node):
  def __init__(self):
    super().__init__('bag_recorder')
    # Suscripciones
    self.image_msg = None
    self.robot_frame_msg = None
    self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
    self.create_subscription(RobotFrame, '/robot_frame', self.robot_frame_callback, 10)

    # Estado
    self.recording = False
    self.writer = None
    self.last_folder = None
    self.start_recording()
    self.timer = self.create_timer(0.1, self.record_messages)

    # Hilos
    """self.keyboard_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
    self.keyboard_thread.start()"""

    self.get_logger().info("Nodo iniciado.")

  # --- Callbacks de tópicos ---
  def image_callback(self, msg):
    self.image_msg = msg

  def robot_frame_callback(self, msg):
    self.robot_frame_msg = msg
  
  # --- Grabación ---
  def start_recording(self):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder_name = f"rosbags/rosbag_{timestamp}"
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
    name='/robot_frame',
    type='RobotFrame/msg/RobotFrame',
    serialization_format='cdr')
    self.writer.create_topic(topic_info)

    self.recording = True
    self.get_logger().info(f"Grabando a 10Hz en carpeta: {folder_name}")

  def record_messages(self):
    """Se ejecuta a 10Hz"""
    now = self.get_clock().now().to_msg()
    timestamp = int(self.get_clock().now().nanoseconds)

    if self.image_msg is not None:
      self.writer.write(
        '/camera/image_raw',
        serialize_message(self.image_msg),
        timestamp
      )

    if self.robot_frame_msg is not None:
      self.writer.write(
        '/robot_frame',
        serialize_message(self.robot_frame_msg),
        timestamp
      )

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
