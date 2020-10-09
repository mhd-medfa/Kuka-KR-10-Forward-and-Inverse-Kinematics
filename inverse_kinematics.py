import numpy as np
from numpy import arctan2, sin, cos
from sympy import symbols, simplify
from sympy.matrices import Matrix
from utils import *
import sys
def inverse_kinematics(xyz, rpy, q=None):
    q_singular, d,a, alpha = KukaKR10R11000_2_configurations()
    q3 = 0
    q4 = 0
    q5 = 0

    # R = simplify(Rx(rpy[0]) * Ry(rpy[1]) * Rz(rpy[2]))
    # T_arr = np.array(R).astype(np.float)

    T = Rz(q[0]) * Tz(d[0]) * Tx(a[0]) * Ry(q[1]) * Tz(a[1]) * Ry(q[2]) * Tx(a[2]) * \
     Tz(d[2]) * Rx(q[3]) * Tx(d[3]) * Ry(q[4]) * Tx(d[4]) * Rx(q[5]) 
    T = simplify(T)
    T_arr = np.array(T).astype(np.float)

    

    #Coordinates of Spherical Wrist Center
    Pc = np.array(xyz) - T_arr[:3,:3].dot(np.array([0,0,d[5]]))
    [xc,yc,zc] = Pc.flatten()

    if xc == yc == 0:
        print("Singularity case")
        return

    q0_1 = arctan2(yc,xc)
    q0_2 = np.pi + arctan2(yc, xc)
    q0 = q0_1

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

    T012 = Rz(q[0]) * Tz(d[0]) * Tx(a[0]) * Ry(q[1]) * Tz(a[1]) * Ry(q[2]) * Tx(a[2]) * \
     Tz(d[2])
    
    T012 = simplify(T012)
    T012_arr = np.array(T012).astype(np.float)

    T345 = Rx(q[3]) * Tx(d[3]) * Ry(q[4]) * Tx(d[4]) * Rx(q[5]) 
    T345 = simplify(T345)
    T345_arr = np.array(T345).astype(np.float)

    T345_bis = T012_arr.T * T_arr

    # Solution 1
    # if abs(T345_bis[2,2]) != 1:
    #     q3 = arctan2(np.float(T345_bis[0,2]), -np.float(T345_bis[1,2]))
    #     q5 = arctan2(np.float(T345_bis[2,0]), np.float(T345_bis[2,0]))
    #     q4 = arctan2(np.sqrt(np.float(T345_bis[0,2])**2 + np.float(T345_bis[1,2])**2), np.float(T345_bis[2,2]))

    # else:
    #     print("Singularity case")
    #     sys.exit(0)

    # Solution 2
    if abs(T345_bis[2,2]) != 1:
        q3 = arctan2(-np.float(T345_bis[0,2]), np.float(T345_bis[1,2]))
        q5 = arctan2(-np.float(T345_bis[2,0]), -np.float(T345_bis[2,0]))
        q4 = arctan2(-np.sqrt(np.float(T345_bis[0,2])**2 + np.float(T345_bis[1,2])**2), np.float(T345_bis[2,2]))

    else:
        print("Singularity case")
        sys.exit(0)

   

    return np.array([q0,q1,q2]).reshape(-1,1), [q3,q4,q5]  