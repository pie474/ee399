"""
Defines all kinematic methods we may use repeatedly over time.

All units are mm or radians.
"""
import sympy as sp
import numpy as np
from sympy import symbols, cos, sin, pi, simplify, lambdify, Matrix, N
from scipy.optimize import least_squares, Bounds


def dh_row_to_transformation(a, alpha, d, theta):
    return Matrix([
        [cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [         0,               sin(alpha),               cos(alpha),              d],
        [         0,                        0,                        0,              1]])


# a values for each link given in mm
a1, a2, a3, a4, a5, a6 = 0, 100, 10, 0, 0, 0

# d values for each link given in mm
# d1, d2, d3, d4, d5, d6 = 136, 0, 0, 107, 0, 65  # obtained via datasheets
d1, d2, d3, d4, d5, d6 = 114, 0, 0, 96, 0, 54 + 95  # obtained experimentally

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
T_GRIPPER = Matrix([
        [1, 0, 0, 95],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

T_TOOL = sp.eye(4)

JOINT_ANGLE_OFFSETS = [0, -pi/2, 0, 0, 0, 0]

JOINT_BOUNDS = (np.radians(np.array([-165, -90, -180, -165, -115, -175])),
                      np.radians(np.array([165, 90, 70, 165, 115, 175])))

shrink = 10
JOINT_BOUNDS = (JOINT_BOUNDS[0] + np.radians(np.ones(6)*shrink),
                JOINT_BOUNDS[1] - np.radians(np.ones(6)*shrink))

T_FORWARD = sp.prod(dh_row_to_transformation(ai, alphai, di, thetai) for ai, alphai, di, thetai in DH_TABLE)
# T_FORWARD = simplify(T_FORWARD)  # takes ~5 seconds to run, uncomment only if needed

def get_translation(transformation):
    return transformation[:3, 3]

def get_rpy(transformation):
    """Returns roll pitch yaw values"""
    return  Matrix([sp.atan2(transformation[2, 1], transformation[2, 2]), \
            sp.asin(-transformation[2, 0]), \
            sp.atan2(transformation[1, 0], transformation[0, 0])])

def get_pose_ts(transformation):
    """x, y, z, yaw, pitch, roll"""
    return Matrix([get_translation(transformation), get_rpy(transformation)])


# angles should be given in rads
def compute_forward_matrix(joint_angles):
    # map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], JOINT_ANGLE_OFFSETS, joint_angles)}

    # Substitute joint angles into the final transformation matrix
    return T_FORWARD.subs(subs_dict)

q_sym = symbols('q1:7')

# need angles in rads
forward_kinematics_func = None

def set_tool(t_tool: Matrix | None = None):
    global T_TOOL, T_FORWARD, forward_kinematics_func, q_sym
    if t_tool:
        T_TOOL = t_tool
    T_FORWARD = sp.prod(dh_row_to_transformation(ai, alphai, di, thetai) for ai, alphai, di, thetai in DH_TABLE) * T_TOOL
    forward_kinematics_func = lambdify(q_sym, compute_forward_matrix(q_sym), 'numpy')

set_tool()

def pose_error(q,  x_target, y_target, z_target, rx_d, ry_d, rz_d):
    angle_weight = 40  # 40
    T_values = forward_kinematics_func(q[0],q[1], q[2], q[3], q[4],q[5])
    roll,pitch,yaw = np.arctan2(T_values[2,1],T_values[2,2]),np.arctan2(-T_values[2,0],
                    np.sqrt(T_values[0,0]**2+T_values[1,0]**2)), np.arctan2(T_values[1,0],T_values[0,0])
    X, Y, Z = T_values[0, 3], T_values[1, 3], T_values[2, 3]
    return [X-x_target, Y-y_target, Z-z_target, (roll-rx_d)*angle_weight, (pitch-ry_d)*angle_weight, (yaw - rz_d)*angle_weight]


# this one works, most important think is that all math should be done in radians and the error should not be split
# into two seperate pieces 
def inverse_kinematics(x_target,y_target,z_target, rx_d, ry_d, rz_d, q_init, max_iterations = 200, tolerance = 1e-8, debug=False):
    q_init_rad = np.clip(np.radians(np.array(q_init)), *JOINT_BOUNDS)
    all_args = (x_target,y_target,z_target, np.radians(rx_d), np.radians(ry_d), np.radians(rz_d))
    solution = least_squares(pose_error, q_init_rad, args = all_args, method = 'trf',
                                        max_nfev = max_iterations, ftol = tolerance, bounds = JOINT_BOUNDS,
                                        # x_scale=(60, 60, 60, 1, 1, 1)
                                        )
    if debug:
        if not solution.success:
            print(f'<<<<<SOLUTION NOT FOUND>>>>>')
        
        iter_thresh = 100
        if solution.nfev > iter_thresh:
            print(f'<<<<<ITERATION THRESHOLD EXCEEDED ({solution.nfev} > {iter_thresh})>>>>>')
        
        bound_dists = np.degrees(np.min([np.abs(np.array(solution.x)-JOINT_BOUNDS[0]), 
                                        np.abs(np.array(solution.x)-JOINT_BOUNDS[1])],
                                        axis=0))
        print(f'IK Solution:           {np.degrees(solution.x)}')
        print(f'Distance from bounds:  {bound_dists}')
        print(f'Residuals:             {solution.fun}')
        print(f'Final cost:            {solution.cost}')

    return np.degrees(solution.x)


if __name__ == '__main__':
    print('main begun')
    q_init = [0.1,0.1,0.1,0.1,0.1,0.1]
    # q_init = [0, 0, 0, 0, 0, 0]


    stream =[[0, 0, 0, 0, 0, 0, 150, 0, 224, 0, 0, 0]
        # [-10.01, 75.05, -87.45, 63.36+ [0, 0, 0], 75.67, 84.99, 212.5, 12.1, 149.1, 133.48, -57.36, 110.23],
        # [-12.39, 72.94, -87.45, 64.77, 87.8, 88.33, 200.6, 8.2, 154.2, 144.1, -60.19, 108.4],
        # [-4.74, 40.86, -3.51, 98.08, 47.54, 68.29, 182.8, 25.5, 115.4, -147.24, -65.36, 10.43],
        # [-9.49, 80.15, -87.36, 60.55, 79.8, 82.7, 211.0, 13.8, 127.1, 145.23, -56.13, 104.99],
        # [-12.74, 80.77, -87.45, 63.72, 92.28, 76.72, 199.9, 6.6, 126.0, 145.91, -57.57, 115.21],
        # [9.31, 73.91, -63.36, -71.36, 73.91, 57.12, 210.1, -17.6, 114.0, -121.28, 47.23, 178.63],
        # [14.32, 80.5, -62.75, -77.78, 85.25, 60.9, 201.5, -5.3, 98.0, -107.7, 42.37, -176.93],
        # [16.43, 78.75, -62.75, -74.97, 95.71, 53.26, 191.5, -0.1, 104.2, -105.5, 34.41, 177.67],
        # [10.89, 86.57, -62.75, -76.28, 75.49, 62.84, 205.9, -14.5, 72.9, -113.59, 39.59, -176.34]
        ]

    for data in stream: 
        x_target, y_target, z_target, rx_d, ry_d, rz_d = data[6], data[7], data[8], data[9], data[10], data[11]
        actual_angles= np.array(data[:6])
        joint_angles = inverse_kinematics(x_target,y_target,z_target,rx_d,ry_d,rz_d,q_init)
        # print('calculated angles: ', np.degrees(joint_angles))
        # print('actual angles: ', actual_angles)
        print('angles:', joint_angles)
        actual_angles = np.radians(actual_angles)
        joint_angles = np.radians(joint_angles)

        print('fwdkin on calc: ', forward_kinematics_func(*joint_angles[:6])[:3,3])
        print('fwdkin on actual: ', forward_kinematics_func(*actual_angles[:6])[:3,3])


# print("Joint Angles (Degrees):")
# print("q1 = ", joint_angles[0], "actual= ", actual_angles[0])
# print("q2 = ", joint_angles[1], "actual= ", actual_angles[1])
# print("q3 = ", joint_angles[2], "actual= ", actual_angles[2])
# print("q4 = ", joint_angles[3], "actual= ", actual_angles[3])
# print("q5 = ", joint_angles[4], "actual= ", actual_angles[4])
# print("q6 = ", joint_angles[5], "actual= ", actual_angles[5])
