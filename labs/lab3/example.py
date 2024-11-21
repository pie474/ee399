import numpy as np
from sympy import symbols, cos, sin, atan2, pi, Matrix, lambdify
from scipy.optimize import least_squares

q1,q2,q3,q4,q5,q6 = symbols('q1 q2 q3 q4 q5 q6')
dh_parameters = [[0,-pi/2,114,q1],
                 [100,0,0,q2-(pi/2)],
                 [10,-pi/2,0,q3],
                 [0,pi/2,96,q4],
                 [0,-pi/2,0,q5],
                 [0,0,136.5,q6]]

#Complete the transformation matrices to represent the operating environment q_orientof the robot
#These should be identity matrices if the arm is operating by itself
T_base = [[1,0,0,0],
          [0,1,0,0],
          [0,0,1,30],
          [0,0,0,1]]
T_tool = [[1,0,0,0],
          [0,1,0,0],
          [0,0,1,195],
          [0,0,0,1]]
T_platform = [[1,0,0,0],
              [0,1,0,0],
              [0,0,1,130],
              [0,0,0,1]]

#Question for discussion: What is the relationship between T_platform and T_tool and T_base?

def get_transformation_matrix(a, alpha, d, theta):
    M = Matrix([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0,0,0,1]])
    return M

T = get_transformation_matrix(dh_parameters[0][0],dh_parameters[0][1],dh_parameters[0][2],dh_parameters[0][3])
for i in range(1,len(dh_parameters)):
    T = T * get_transformation_matrix(dh_parameters[i][0],dh_parameters[i][1],dh_parameters[i][2],dh_parameters[i][3])

q_sym = symbols('q1:7')

def symbolic_forward_kinematics(q_values):
    T_symbolic = T.subs([(q1 , q_values[0]),(q2 , q_values[1]),(q3 , q_values[2]),(q4 , q_values[3]),
            (q5 , q_values[4]),(q6 , q_values[5])])
    return T_symbolic

forward_kinematics_func = lambdify(q_sym, symbolic_forward_kinematics(q_sym), 'numpy')

def position_error(q_position, x_target, y_target, z_target, link_lengths):
    T_values = forward_kinematics_func(q_position[0],q_position[1],q_position[2], 0,0,0)
    X,Y,Z = T_values[0][3], T_values[1][3], T_values[2][3]
    return [X-x_target, Y-y_target, Z-z_target]

def orientation_error(q_orientation, rx_d, ry_d, rz_d):
    T_values = forward_kinematics_func(0,0,0,q_orientation[0],q_orientation[1],q_orientation[2])
    # roll,pitch,yaw = np.arctan2(z,y), np.arctan2(-x,np.sqrt(x**2+y**2)), np.arctan2(x,y)
    roll,pitch,yaw = np.arctan2(T_values[2][1],T_values[2][2]),np.arctan2(-T_values[2][0],
                    np.sqrt(T_values[0][0]**2+T_values[1][0]**2)), np.arctan2(T_values[1][0],T_values[0][0])
    return [roll-rx_d, pitch-ry_d, yaw - rz_d]

#Assumptions?: 

def inverse_kinematics(x_target,y_target,z_target, rx_d, ry_d, rz_d, q_init, link_lengths, max_iterations = 100, tolerance = 1e-6):
    position_args = (x_target,y_target,z_target,link_lengths)
    q_position_solution = least_squares(position_error, q_init[:3], args = position_args, method = 'lm', 
                                        max_nfev = max_iterations, ftol = tolerance).x
    
    orientation_args = (rx_d,ry_d,rz_d)
    q_orientation_solution = least_squares(orientation_error, q_init[3:], args = orientation_args, method = 'lm',
                                        max_nfev = max_iterations, ftol = tolerance).x

    joint_angles = np.concatenate((q_position_solution, q_orientation_solution))

    return joint_angles

# x_target = 200
# y_target = 100
# z_target = 150
# rx_d = .5
# ry_d = 0.2
# rz_d = 0.3

q_init = [0.1,0.1,0.1,0.1,0.1,0.1]
link_lengths = [114,100,10,96,56.5]

x_target = 182.9
y_target = -17.4
z_target = 180.2
rx_d = 102.42
ry_d = -3.74
rz_d = 84.15
# q_init = [0.1,0.1,0.1,0.1,0.1,0.1]
# link_lengths = [114,100,10,96,56.5]

x_target, y_target, z_target, rx_d, ry_d, rz_d = 212.5, 12.1, 149.1, 133.48, -57.36, 110.23

joint_angles = inverse_kinematics(x_target,y_target,z_target,rx_d,ry_d,rz_d,q_init,link_lengths)

joint_angles = np.degrees(joint_angles)

print("Joint Angles (Degrees):")
print("q1 = ", joint_angles[0])
print("q2 = ", joint_angles[1])
print("q3 = ", joint_angles[2])
print("q4 = ", joint_angles[3])
print("q5 = ", joint_angles[4])
print("q6 = ", joint_angles[5])