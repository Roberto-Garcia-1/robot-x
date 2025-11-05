
from sympy import symbols, pi, Matrix, cos, sin, lambdify, asin
import math


class RobotModel():

  def __init__(self):
    z_0_1 = 0.105
    L1 = 0.084
    L2 = 0.084
    L3 = 0.115
    theta_1, theta_2, theta_3, theta_4 = symbols('theta_1, theta_2, theta_3, theta_4')
    T_0_1 = self.trans_homo_xz(0, z_0_1, 0, theta_1)
    T_1_2 = self.trans_homo_xz(0, 0, -pi/2, theta_2 - pi/2)
    T_2_3 = self.trans_homo_xz(L1, 0, 0, theta_3)
    T_3_4 = self.trans_homo_xz(L2, 0, 0, theta_4)
    T_4_p = self.trans_homo_xz(L3, 0, -pi/2, 0)

    T_0_p = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_p
    self.T_0_p_lam = lambdify([theta_1, theta_2, theta_3, theta_4], T_0_p)

  def trans_homo_xz(self, x=0, z=0, gamma=0, alpha=0)->Matrix:
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_z

    
  def direct_kinematics(self, th1, th2, th3, th4):
    p1 = self.T_0_p_lam(th1, th2, th3, th4)
    x = float(p1[0,3])
    y = float(p1[1,3])
    z = float(p1[2,3])
    bet = float(asin(p1[0,2]))
    al= float(asin(-p1[0,1] / cos(bet)))
    gam = float(asin(-p1[1,2] / cos(bet)))
    return (x, y, z, gam, bet, al)