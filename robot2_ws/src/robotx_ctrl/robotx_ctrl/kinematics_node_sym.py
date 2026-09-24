#!/usr/bin/env python3
import rclpy
from sympy import atan2
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import JointState
from robotx_ctrl.kinematics import Robot
class NodoSuscriptor(Node):
  def __init__(self):
    super().__init__("servidor")
    # Inicializar robot 
    self.get_logger().info("Inicializando robot")
    self.robot = Robot(l=(0.3, 0.3, 0.3))
    self.get_logger().info("Inicializado")
    # Suscriptor
    self.twist_sub = self.create_subscription(Twist,"/efector", self.tw_callback, 10)
    # PointStamped
    self.ps_sub = self.create_subscription(PointStamped,"/clicked_point", self.ps_callback, 10)
    # Publicador de posiciones
    self.pub = self.create_publisher(JointState, "/joint_states", 10)
    # Estado del movimiento
    self.moving = False

  # PointStamped callback
  def ps_callback(self, msg:PointStamped):
    if self.moving:
      return
    self.moving = True
    self.get_logger().info("Recibida posición")
    self.get_logger().info(str(msg.point))
    # Llamar cinemática inversa
    try:
      gl = self.joint_goals.position
    except:
      gl = (0, 0.1, 0.1, 0.1, 0)
    al = atan2(msg.point.y, msg.point.x)
    if abs(msg.point.z) < 0.1:
      self.robot.def_trayectoria(t_f = 2, frec = 15, 
      th_i=(gl[0], gl[1], gl[2], gl[3]), 
      xi_fn = (0.11, 0.20, 0, al))
    else:  
      self.robot.def_trayectoria(t_f = 2, frec = 15, th_i=(gl[0], gl[1], gl[2], gl[3]), xi_fn = (msg.point.x, msg.point.z, 0, gl[0]))
    self.get_logger().info(str(self.robot.th_m[:, self.robot.muestras - 1]))
    # Implementar timer para publicar periódicamente la posición de las juntas
    self.count = 0
    self.joint_goals =  JointState()
    self.joint_goals.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint5a1"]
    self.get_logger().info("Publicando trayectoria de las juntas")
    self.position_publisher_timer = self.create_timer(self.robot.dt, 
                                                       self.trayectory_publisher_callback)
  # Twist
  def tw_callback(self, msg:Twist):
    if self.moving:
      return
    self.moving = True
    self.get_logger().info("Recibida posición")
    # Llamar cinemática inversa
    self.robot.def_trayectoria(t_f = 2, frec = 15, th_i=(0.1, 0.1, 0.1), xi_fn = (msg.linear.x, msg.linear.z, msg.angular.y))
    self.get_logger().info(str(self.robot.th_m[:, self.robot.muestras - 1]))
    # Implementar timer para publicar periódicamente la posición de las juntas
    self.count = 0
    self.joint_goals =  JointState()
    self.joint_goals.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
    self.get_logger().info("Publicando trayectoria de las juntas")
    self.position_publisher_timer = self.create_timer(self.robot.dt, 
                                                       self.trayectory_publisher_callback)
    
  def trayectory_publisher_callback(self):
    # Marca de tiempo
    self.joint_goals.header.stamp = self.get_clock().now().to_msg()
    # Obtener valores de las juntas de las matrices de muestreo
    th1 = float(self.robot.th_m[0, self.count])
    th2 = float(self.robot.th_m[1, self.count])
    th3 = float(self.robot.th_m[2, self.count])
    th4 = float(self.robot.th_m[3, self.count])
    # Asignar valor al mensaje
    self.joint_goals.position = [th1, th2, th3, th4, float(0.0), float(0.0)]
    # Publicar
    self.pub.publish(self.joint_goals)
    self.count+=1
    if (self.count >= len(self.robot.th_m[0,:])):
      self.count = 0
      self.position_publisher_timer.cancel()
      self.get_logger().info("Trayectoria finalizada")
      self.moving = False
  

def main():
  try:
    rclpy.init()
    
    nodo_suscriptor = NodoSuscriptor()
    rclpy.spin(nodo_suscriptor)

    rclpy.shutdown()
  except KeyboardInterrupt as key_ex:
    print(key_ex)


if __name__ == "__main__":
  main()

