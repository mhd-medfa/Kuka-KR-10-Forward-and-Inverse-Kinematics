import numpy as np

from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
from utils import *

def forward_kinematics(q = None):
  """
  It returns position and orientation of end-effector based on q angles of joints
  """
  q_singular, d,a, alpha = KukaKR10R11000_2_configurations()

  if q == None:
    q = q_singular


  T = Rz(q[0]) * Tz(d[0]) * Tx(a[0]) * Ry(q[1]) * Tz(a[1]) * Ry(q[2]) * Tx(a[2]) * \
     Tz(d[2]) * Rx(q[3]) * Tx(d[3]) * Ry(q[4]) * Tx(d[4]) * Rx(q[5]) 
  T = simplify(T)
  T_arr = np.array(T).astype(np.float)

  if abs(T_arr[0,2]) != 1:
        r = np.arctan2(T_arr[2,1], T_arr[2,2])
        p = np.arctan2(-T_arr[2,1], np.sqrt(T_arr[0,0]**2 + T_arr[1,0]**2))
        y = np.arctan2(T_arr[1,0], T_arr[0,0])
  else:
      print("Singularity case")
      return

  return T_arr[:3,3].flatten(), [r,p,y]



def forward_kinematics_old():
  ### Create symbols for joint  variables
  q1, q2, q3, q4, q5, q6 = symbols('q1:7')  # q_i
  l1, l2, l3, l4, l5, l6 = symbols('l1:7')  # link_offset_i
  a0, a1, a2, a3, a4, a5 = symbols('a0:6')  # link_length_i
  alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha0:6')  # link_twist_i


  ### Kuka KR 10 R1100-2 ###
  # DH Parameters
  DH_params = {alpha0: -pi/2, a0:      0, l1:  400,
              alpha1:     0, a1:    560, l2:     0,  q2: q2-pi/2,
              alpha2:  pi/2, a2:     25, l3:     0,
              alpha3: -pi/2, a3:      0, l4:   515,
              alpha4:  pi/2, a4:      0, l5:     0,
              alpha5:     0, a5:      0, l6:    90,
              }

  # ### Kuka KR 10 R1100-2 ###
  # # DH Parameters
  # DH_params =  {alpha0:     0, a0:      0, l1:  400,
  #      alpha1: -pi/2, a1:   560, l2:     0,  q2: q2-pi/2,
  #      alpha2:     0, a2:    25, l3:     0,
  #      alpha3: -pi/2, a3:     0, l4:  515,
  #      alpha4:  pi/2, a4:      0, l5:     0,
  #      alpha5: -pi/2, a5:      0, l6:     90 }

  def TransformationMatrix(q, l, a, alpha):
    """
    Create Transformation Matrix according to Modified DH convention
    q: angle from X_i-1 to X_i  about Z_i
    l: distance from X_i-1 to X_i  along Z_i
    a: distance from Z_i to Z_i+1 along X_i
    alpha: angle from Z_i to Z_i+1 about X_i
    """
    T = Matrix([[cos(q), -sin(q), 0., a],
                  [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*l],
                  [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*l],
                  [0., 0., 0., 1.]])
    return T

  #### Homogeneous Transforms
  # base_link to link1    
  T0_1 = TransformationMatrix(q1, l1, a0, alpha0)
  # T0_1

  T0_1 = T0_1.subs(DH_params)
  # T0_1

  T1_2 = TransformationMatrix(q2, l2, a1, alpha1)
  T1_2 = T1_2.subs(DH_params)

  T2_3 = TransformationMatrix(q3, l3, a2, alpha2)
  T2_3 = T2_3.subs(DH_params)

  T3_4 = TransformationMatrix(q4, l4, a3, alpha3)
  T3_4 = T3_4.subs(DH_params)

  T4_5 = TransformationMatrix(q5, l5, a4, alpha4)
  T4_5 = T4_5.subs(DH_params)

  T5_Tool = TransformationMatrix(q6, l6, a5, alpha5)
  T5_Tool = T5_Tool.subs(DH_params)

  T0_2 = simplify(T0_1 * T1_2) # base_link to link 2
  T0_3 = simplify(T0_2 * T2_3) # base_link to link 3
  # T0_3

  T3_5 = simplify(T3_4 * T4_5) # base_link to link 5
  T3_Tool = simplify(T3_5 * T5_Tool) # base_link to link Tool
  # T3_Tool

  T0_4 = simplify(T0_3 * T3_4) # base_link to link 4
  T0_5 = simplify(T0_4 * T4_5) # base_link to link 5
  T0_Tool = simplify(T0_5 * T5_Tool) # base_link to link Tool

  # T0_Tool
  #pi/2,pi/4,pi/4,0,0,0
  print(T0_Tool.evalf(subs={q1:pi/2, q2:pi/4, q3:pi/4, q4:0, q5:0, q6:0}))

  R0_3 = T0_3[0:3,0:3]
  R0_Tool = T0_Tool[0:3,0:3]
  # R0_3

  R3_6 = simplify(R0_3.transpose()*R0_Tool)
  # R3_6

  # T3_Tool

q_test = [pi/2,pi/4,pi/4,0,0,0]
print(forward_kinematics(q_test))