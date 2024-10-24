"""
Defines all kinematic methods we may use repeatedly over time.

All units are in mm.
"""
import sympy as sp
from sympy import symbols, cos, sin, pi, simplify, pprint, Matrix


def get_transformation_matrix(a, alpha, d, theta):
    return Matrix([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]])


# a values for each link given in mm
a1, a2, a3, a4, a5, a6 = 0, 100, 0, 0, 0, 0

# d values for each link given in mm
d1, d2, d3, d4, d5, d6 = 136, 0, 0, 107, 0, 65

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

# Define the offset parameters
JOINT_ANGLE_OFFSETS = [0, -pi/2, 0, 0, 0, 0]

T0EE = sp.prod(get_transformation_matrix(ai, alphai, di, thetai) for ai, alphai, di, thetai in DH_TABLE)
# T0EE = simplify(T0EE)  # takes like 5 seconds to run

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
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], JOINT_ANGLE_OFFSETS, joint_angles)}

    # Substitute joint angles into the final transformation matrix
    return simplify(T0EE.subs(subs_dict))


def cyclic_coordinate_descent(T0EE_d, curr_joint_angles, state_func, error_threshold=1, max_iterations=1000):
    """Implements the Cyclic Coordinate Descent (CCD) algorithm to numerically find a solution to the 
        inverse kinematics problem given by the target base-EE transformation

    """
    state_d = state_func(T0EE_d)
    joint_angles = curr_joint_angles.copy()

    def error(ja):
        return (state_func(compute_forward_matrix(ja)) - state_d).norm()

    while(error(joint_angles) > error_threshold):
        ...


Td_test = Matrix([[1, 0, 0, 100],
                  [0, 1, 0, 100],
                  [0, 0, 1, 100],
                  [0, 0, 0, 1]])


pprint(get_pose_ts(Td_test).norm())