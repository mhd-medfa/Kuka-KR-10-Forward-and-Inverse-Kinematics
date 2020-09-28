import numpy as np
from numpy import arctan2, sin, cos
from sympy import symbols, simplify
from sympy.matrices import Matrix
from utils import *

def inverse_kinematics(xyz, rpy):
    q_singular, d,a, alpha = KukaKR10R11000_2_configurations()

    R = simplify(Rx(rpy[0]) * Ry(rpy[1]) * Rz(rpy[2]))
    R_arr = np.array(R).astype(np.float)
    #Coordinates of Spherical Wrist Center
    Pc = np.array(xyz) - R_arr[:3,:3].dot(np.array([0,0,d[5]]))
    [xc,yc,zc] = Pc.flatten()

    if xc == yc == 0:
        print("Singularity case")
        return

    q0 = arctan2(yc,xc)
    
    l = sqrt(xc**2 + yc**2) - a[0]
    h = zc - d[0]
    u = sqrt(l**2+h**2)
    delta_q = arctan2(d[3], a[2])
    a2_prime = sqrt(a[2]**2 + d[3]**2)
    A = (a[1]**2+a2_prime**2-u**2) / (2.0*a[1]*a2_prime)
    A = np.float(A)
    A_sin = np.float(sqrt(1-A**2))
    psi1 = arctan2(A_sin, A)
    psi2 = arctan2(-A_sin, A)
    psi = psi1
    q2 = np.pi - psi - delta_q


    phi2 = arctan2(np.int(h), np.int(l))

    B = sin(psi1) * a2_prime / u
    B = np.float(B)
    B_cos = np.float(sqrt(1-B**2))
    phi1_1 = arctan2(B, B_cos)
    phi1_2 = arctan2(B, -B_cos)

    phi1 = phi1_1

    q1 = phi1 + phi2

    q3 = arctan2(np.float(R_arr[1,2]), np.float(R_arr[0,2]))
    q4 = arctan2(np.float(R_arr[1,2]), np.float(-R_arr[2,2] * sin(q3)))
    q5 = arctan2(np.float(-R_arr[2,1]), np.float(R_arr[2,0]))
    # q3 = arctan2(np.float(R_arr[1,0]), np.float(-R_arr[2,0]))
    # q5 = arctan2(np.float(-R_arr[0,1]), np.float(R_arr[0,2]))
    # if abs(np.float(R_arr[0,0]) - 1) <= 0.0001 :
    #     if abs(np.float(R_arr[0,2])) <= 0.0001:
    #         q4 = arctan2(np.float(R_arr[0,1])/sin(q5), np.float(R_arr[0,0]))
    #     else:
    #         q4 = arctan2(np.float(R_arr[0,2])/cos(q5), np.float(R_arr[0,0]))

    # else:   
    #     q4 = np.arccos(np.float(R_arr[0,0]))
   

    return np.array([q0,q1,q2]).reshape(-1,1), [q3,q4,q5]  