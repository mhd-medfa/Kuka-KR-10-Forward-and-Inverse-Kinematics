from numpy import pi
from forward_kinematics import forward_kinematics
from inverse_kinematics import inverse_kinematics


case1 = [pi/2,pi/12,pi/10,0,0,0]
case2 = [pi/2,pi/2,pi/6,0,0,0]
test = case1
print("FK function:")
position, rotation = forward_kinematics(test)
print("FK pos:", position)

print()
print("IK function:")
arm_joints_angles, end_effector_joints_angles = inverse_kinematics(position, rotation)
print("end-effector:\n ", end_effector_joints_angles)
[q0,q1,q2] = arm_joints_angles[:3,0]
i_position,i_rotation = forward_kinematics([q0,q1,q2,0,0,0])
print("Inverse Kinematics result: ", arm_joints_angles.flatten())  
print("Expected:", test)
print("Forward Kinematics position:", position)
print("Inverse Kinematics Position", i_position)