import numpy as np
from scipy.spatial.transform import Rotation as Rot
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

def rotation_to_transformation_matrix(R):
    """
    It converts from 3x3 Rotation matrix to 4x4 Transformation matrix
    """
    R = Matrix(R)
    T = R.col_insert(3, Matrix([0., 0., 0.]))
    T = T.row_insert(3, Matrix([[0., 0., 0., 1.]]))
    return T
    
def Rx(q):
    R = Matrix(Rot.from_euler('x', q).as_matrix())

    return rotation_to_transformation_matrix(R)

def Ry(q):
    R = Matrix(Rot.from_euler('y', q).as_matrix())
    return rotation_to_transformation_matrix(R)

def Rz(q):
    R = Matrix(Rot.from_euler('z', q).as_matrix())
    return rotation_to_transformation_matrix(R)

def Tx(d):
    T = np.eye(4,4)
    T[0,3] = np.float(d)
    T = Matrix(T)
    return T

def Ty(d):
    T = np.eye(4,4)
    T[1,3] = np.float(d)
    T = Matrix(T)
    return T

def Tz(d):
    T = np.eye(4,4)
    T[2,3] = np.float(d)
    T = Matrix(T)
    return T

def KukaKR10R11000_2_configurations():
    q = [0,-pi/2,0,0,0,0]
    d = [400,0,0,515,0,90]
    a = [25,560,25,0,0,0]
    alpha = [-pi/2,0,-pi/2,pi/2,-pi/2,0]
    return q, d,a, alpha

if __name__ == "__main__":
    q = 1
    print(Rot.from_euler('x', q).as_matrix())

    print("=============")
    print(rotation_to_transformation_matrix(Rot.from_euler('x', q).as_matrix()))