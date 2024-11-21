"""
Defines all kinematic methods we may use repeatedly over time.

All units are mm or radians.
"""
import sympy as sp
import numpy as np
from sympy import symbols, cos, sin, pi, simplify, lambdify, Matrix, N
from scipy.optimize import least_squares


def dh_row_to_transformation(a, alpha, d, theta):
    return Matrix([
        [cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [         0,               sin(alpha),               cos(alpha),              d],
        [         0,                        0,                        0,              1]])


# a values for each link given in mm
a1, a2, a3, a4, a5, a6 = 0, 100, 0, 0, 0, 0

# d values for each link given in mm
# d1, d2, d3, d4, d5, d6 = 136, 0, 0, 107, 0, 65  # obtained via datasheets
d1, d2, d3, d4, d5, d6 = 120, 0, 0, 100, 0, 50  # obtained experimentally

# joint angles, left symbolic so they can be substituted later
q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

# Formatted as [a, alpha d, theta]
DH_TABLE = [
    [a1, -pi/2, d1, q1],  # J1
    [a2,     0, d2, q2],  # J2
    [a3, -pi/2, d3, q3],  # J3
    [a4,  pi/2, d4, q4],  # J4
    [a5, -pi/2, d5, q5],  # J5
    [a6,     0, d6, q6]   # J6 (End effector)
]

JOINT_ANGLE_OFFSETS = [0, -pi/2, 0, 0, 0, 0]

T_FORWARD = sp.prod(dh_row_to_transformation(ai, alphai, di, thetai) for ai, alphai, di, thetai in DH_TABLE)
# T_FORWARD = simplify(T_FORWARD)  # takes ~5 seconds to run, uncomment only if needed

def get_translation(transformation):
    return transformation[:3, 3]

def get_ypr(transformation):
    """Returns yaw pitch roll values"""
    return  Matrix([sp.atan2(transformation[1, 0], transformation[0, 0]), \
            sp.asin(transformation[2, 0]), \
            sp.atan2(transformation[2, 1], transformation[2, 2])])

def get_pose_ts(transformation):
    """x, y, z, yaw, pitch, roll"""
    return Matrix([get_translation(transformation), get_ypr(transformation)])

def compute_forward_matrix(joint_angles):
    # map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle*pi/180 for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], JOINT_ANGLE_OFFSETS, joint_angles)}

    # Substitute joint angles into the final transformation matrix
    return simplify(T_FORWARD.subs(subs_dict))

q_sym = symbols('q1:7')

forward_kinematics_func = lambdify(q_sym, compute_forward_matrix(q_sym), 'numpy')

def position_error(q_position, x_target, y_target, z_target, link_lengths):
    # Calculate forward kinematics for the first three links
    T_values = forward_kinematics_func(q_position[0], q_position[1], q_position[2], 0, 0, 0)
    # Extract the end-effector position from the transformation matrix
    X, Y, Z = T_values[0,3], T_values[1,3], T_values[2,3]
    # return an array of differences between calculated and target positions
    return [x_target - X, y_target - Y, z_target - Z]

def orientation_error(q_orientation, rx_d, ry_d, rz_d):
    # Calculate forward kinematics for the last three links
    T_values = forward_kinematics_func(0, 0, 0, q_orientation[0], q_orientation[1], q_orientation[2])
    # Complete the trig equations
    #  Extract the end-effector orientation from the transformation matrix
    roll, pitch, yaw = np.arctan2(T_values[2][1],T_values[2][2]), \
                        np.arctan2(-T_values[2][0], np.sqrt(T_values[0][0]**2+T_values[1][0]**2)), \
                        np.arctan2(T_values[1][0],T_values[0][0])
    # yaw, pitch, roll = get_ypr(T_values)
    #return an array of differences between calculated and target orientations
    return [rx_d - roll, ry_d - pitch, rz_d - yaw]


def inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, q_init, link_lengths, max_iterations=100, tolerance=1e-6):
    # Perform numerical inverse kinematics for position
    position_args = (x_target, y_target, z_target, link_lengths)
    q_position_solution = least_squares(position_error, q_init[0:3], args=position_args, \
        method='lm', max_nfev=max_iterations, ftol=tolerance).x
    # Perform numerical inverse kinematics for orientation
    orientation_args = (rx_d, ry_d, rz_d)
    q_orientation_solution = least_squares(orientation_error, q_init[3:6], args=orientation_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x
    # discussion question: We have discussed the concept of Jacobian Matrix in lecture and this document. Where do you think the Jacobian matrix is implemented in this code?
    # Combine the position and orientation components to get the final joint angles
    joint_angles = np.concatenate((q_position_solution, q_orientation_solution))
    return np.degrees(joint_angles) % 360

if __name__ == '__main__':
    print('main begun')
    qs = np.array([-4.74, 26.27, -64.16, 13.79, 90.52, 75.84])
    qs = np.array([-10.01, 75.05, -87.45, 63.36, 75.67, 84.99])

    # x_target = 200
    # y_target = 0
    # z_target = 0
    # rx_d = 102.42
    # ry_d = -3.74
    # rz_d = 84.15
    x_target, y_target, z_target, rx_d, ry_d, rz_d = 212.5, 12.1, 149.1, 133.48, -57.36, 110.23
    sol = inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, qs, None)
    print(N(get_translation(compute_forward_matrix(Matrix(sol)))))
    print([x_target, y_target, z_target])
    # print(np.array([-10.01, 75.05, -87.45, 63.36, 75.67, 84.99]) % 360)