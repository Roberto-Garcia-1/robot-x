from sympy import *
import matplotlib.pyplot as plt
class Robot():
  def __init__(self, l:tuple[int]):
    # Cinemática directa
    th1, th2, th3, th4 = symbols("theta_1, theta_2, theta_3, theta_4")
    self.th1, self.th2, self.th3, self.th4 = th1, th2, th3, th4
    T_0_1 = self._tr_h(alpha = th1, z = 0.0607)
    T_1_2 = self._tr_h(y = -0.04145, gamma = -pi/2,alpha = -pi/2 + th2)
    T_2_3 = self._tr_h(x = 0.08285, alpha = th3)
    T_3_4 = self._tr_h(x = 0.08285, alpha = th4)
    T_4_p = self._tr_h(x = 0.15)
    T_0_p = simplify(T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_p)
    xi_0_p = Matrix([T_0_p[0, 3],
                     T_0_p[2, 3],
                     th2 + th3 + th4 - pi/2,
                     th1])
    J = Matrix([[diff(xi_0_p, th1),
                 diff(xi_0_p, th2),
                 diff(xi_0_p, th3),
                 diff(xi_0_p, th4)]])
    J_inv = J.inv()
    x_dot, y_dot, gamma_dot, beta_dot = symbols("x_dot, y_dot, gamma_dot, beta_dot")
    self.x_dot, self.y_dot, self.gamma_dot, self.beta_dot = x_dot, y_dot, gamma_dot, beta_dot
    xi_0_p_dot = Matrix([x_dot, y_dot, gamma_dot, beta_dot])
    th_dot = J_inv * xi_0_p_dot
    self.xi_0_p = xi_0_p
    self.th_dot = th_dot
    # Construir una trayectoria
    t = symbols("t")
    self.t = t
    a_0, a_1, a_2, a_3, a_4, a_5 = symbols("a_0, a_1, a_2, a_3, a_4, a_5")
    self.a_0, self.a_1, self.a_2, self.a_3, self.a_4, self.a_5 = a_0, a_1, a_2, a_3, a_4, a_5
    self.lam = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    # derivadas del polinomio
    self.lam_dot = diff(self.lam, t)
    self.lam_dot_dot = diff(self.lam_dot, t)

  def def_trayectoria(self, t_f=2, frec=30, th_i=(0, 0.1, 0.1, 0.1), xi_fn=(0.2, 0.25, 0, 0)):
    xi_i = self.xi_0_p.subs({self.th1: th_i[0], 
                              self.th2: th_i[1], 
                              self.th3: th_i[2],
                              self.th4: th_i[3]})
    self.dt = 1.0/frec
    self.muestras = t_f * frec + 1
    eq1 = self.lam.subs({self.t: 0})
    eq2 = self.lam.subs({self.t: t_f}) - 1
    eq3 = self.lam_dot.subs({self.t: 0})
    eq4 = self.lam_dot.subs({self.t: t_f})
    eq5 = self.lam_dot_dot.subs({self.t: 0})
    eq6 = self.lam_dot_dot.subs({self.t: t_f})
    solutions = solve((eq1, eq2, eq3, eq4, eq5, eq6),
                    (self.a_0, self.a_1, self.a_2, self.a_3, self.a_4, self.a_5))
    lam_s = self.lam.subs(solutions)
    lam_dot_s = self.lam_dot.subs(solutions)
    lam_dot_dot_s = self.lam_dot_dot.subs(solutions)
    
    # Posición, velocidad y aceleración
    xi_f = Matrix([xi_fn[0], xi_fn[1], xi_fn[2], xi_fn[3]])
    xi_eq         = xi_i + (xi_f - xi_i) * lam_s
    xi_dot_eq     = (xi_f - xi_i) * lam_dot_s
    xi_dot_dot_eq = (xi_f - xi_i) * lam_dot_dot_s

    # Generar arreglos para guardar muestreo
    # Tiempo
    t_m = Matrix.zeros(1, self.muestras)
    for i in range(self.muestras):
      t_m[i] = self.dt * i
    t_m
    # Arreglos para posición, velocidad y aceleración del E.F.
    xi_m         = Matrix.zeros(4, self.muestras)
    xi_dot_m     = Matrix.zeros(4, self.muestras)
    xi_dot_dot_m = Matrix.zeros(4, self.muestras)
    # Muestreo
    for i in range(self.muestras):
      xi_m[:, i]         = xi_eq.        subs({self.t: t_m[i]})
      xi_dot_m[:, i]     = xi_dot_eq.    subs({self.t: t_m[i]})
      xi_dot_dot_m[:, i] = xi_dot_dot_eq.subs({self.t: t_m[i]})
    
    self.xi_m = xi_m
    self.t_m = t_m
    print(self.xi_m)
    #Arreglos para posición, velocidad y aceleración de las juntas
    th_m = Matrix.zeros(4, self.muestras)
    th_dot_m = Matrix.zeros(4, self.muestras)
    th_dot_dot_m = Matrix.zeros(4, self.muestras)
    th_m[:, 0] = Matrix([th_i[0], th_i[1], th_i[2], th_i[3]])
    # Cinemática inversa
    th_dot_l = lambdify([self.th1, self.th2, self.th3, self.th4,
                        self.x_dot, self.y_dot, self.gamma_dot, self.beta_dot],
                        self.th_dot, 'numpy')
    for i in range(self.muestras):
      """th_dot_m[:, i] = (self.th_dot.subs({self.th1: th_m[0, i], self.th2: th_m[1, i], 
                                    self.th3: th_m[2, i], self.th4: th_m[3, i], 
                                    self.x_dot: xi_dot_m[0, i],
                                    self.y_dot: xi_dot_m[1, i], 
                                    self.gamma_dot: xi_dot_m[2, i],
                                    self.beta_dot: xi_dot_m[2, i]})).evalf()"""
      th_dot_m[:, i] = th_dot_l(float(th_m[0, i]), float(th_m[1, i]), 
                                    float(th_m[2, i]), float(th_m[3, i]), 
                                    float(xi_dot_m[0, i]),
                                    float(xi_dot_m[1, i]), 
                                    float(xi_dot_m[2, i]),
                                    float(xi_dot_m[3, i]))
      print(i)
      if i < self.muestras - 1:
        th_m[:, i+1] = th_m[:, i] + th_dot_m[:, i] * self.dt
      if i > 0:
        th_dot_dot_m[:, i-1] = (th_dot_m[:, i] - th_dot_m[:, i-1])/self.dt
    th_dot_dot_m
    self.th_m = th_m
    self.xi_m = xi_m
    self.t_m = t_m
    self.graficar_xi()
    self.graficar_th()
  def graficar_theta(self):
    pass
  def graficar_xi(self):
    fig, (x_g, y_g, ga_g, be_g) = plt.subplots(nrows = 1, ncols = 4)
    fig.suptitle("Posiciones del efector final")
    x_g.set_title("x")
    y_g.set_title("y")
    ga_g.set_title("gamma")
    be_g.set_title("beta")
    x_g.plot(self.t_m.T, self.xi_m[0, :].T, color="RED")
    y_g.plot(self.t_m.T, self.xi_m[1, :].T, color="green")
    ga_g.plot(self.t_m.T, self.xi_m[2, :].T, color=(0,0,1))
    be_g.plot(self.t_m.T, self.xi_m[3, :].T, color=(0,1,1))
    plt.show()
    pass
  def graficar_th(self):
    fig, (th1_g, th2_g, th3_g, th4_g) = plt.subplots(nrows = 1, ncols = 4)
    fig.suptitle("Posiciones de las juntas")
    th1_g.set_title("th1")
    th2_g.set_title("th2")
    th3_g.set_title("th3")
    th4_g.set_title("th4")
    th1_g.plot(self.t_m.T, self.th_m[0, :].T, color="RED")
    th2_g.plot(self.t_m.T, self.th_m[1, :].T, color="green")
    th3_g.plot(self.t_m.T, self.th_m[2, :].T, color=(0,0,1))
    th4_g.plot(self.t_m.T, self.th_m[3, :].T, color=(0,1,1))
    plt.show()
    pass

  def _tr_h(self, x=0, y=0, z=0, gamma=0, beta=0, alpha=0):
    T_x = Matrix([[1,          0,           0, x],
                  [0, cos(gamma), -sin(gamma), 0],
                  [0, sin(gamma),  cos(gamma), 0],
                  [0,          0,           0, 1],])
    T_y = Matrix([[cos(beta),  0, sin(beta), 0],
                  [        0,  1,         0, y],
                  [-sin(beta), 0, cos(beta), 0],
                  [0, 0, 0, 1]])
    T_z = Matrix([[cos(alpha), -sin(alpha), 0, 0], 
                  [sin(alpha),  cos(alpha), 0, 0],
                  [         0,           0, 1, z], 
                  [         0,           0, 0, 1]])
    return T_x * T_y * T_z
    
def main():
  robot = Robot(l=(0.1, 0.1, 0.1))
  robot.def_trayectoria()
if __name__ == "__main__":
  main()