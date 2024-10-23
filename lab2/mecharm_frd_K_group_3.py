from sympy import symbols, cos, sin, pi, simplify, pprint
import sympy as sp
from sympy.matrices import Matrix
import numpy as np
import math

q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')
START = [0, 0, 0, 0, 0, 0]


def get_transformation_matrix(a, alpha, d, theta):
    M = Matrix([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]])
    return M



# a values for each link given in mm
a1, a2, a3, a4, a5, a6 = 0, 100, 0, 0, 0, 0

# d values for each link given in mm
d1, d2, d3, d4, d5, d6 = 136, 0, 0, 107, 0, 65
# q1, q2, q3, q4, q5, q6 = [0, -pi/2, 0, 0, 0, 0]

# Formatted as [a, alpha d, theta]
dh_params = [
    [a1, -pi/2, d1, q1],  # J1
    [a2, 0, d2, q2],      # J2
    [a3, -pi/2, d3, q3],     # J3
    [a4, pi/2, d4, q4],   # J4
    [a5, -pi/2, d5, q5],    # J5
    [a6, 0, d6, q6]       # J6 (End effector)
]

Tae = []
for ai, alphai, di, thetai in dh_params:
    Tae.append(get_transformation_matrix(ai, alphai, di, thetai))
#
# i = 1
# for t in Tae:
#     print("A" + str(i))
#     i += 1
#     pprint(t)


# print("FINAL T")
Tfinal = Tae[0]*Tae[1]*Tae[2]*Tae[3]*Tae[4]*Tae[5]
# sp.pprint(Tfinal)
#
# print(Tfinal*sp.Matrix([1, 1, 1, 1]))

a, b, c, d, e, f = [-90, 0, 0, 0, 0, 0]
joint_angles = [np.radians(a), np.radians(b), np.radians(c), np.radians(d), np.radians(e), np.radians(f)]
# Define the offset parameters. Complete the values
offsets = [0, -pi/2, 0, 0, 0, 0]
# map the joint angles and offsets into the transformation matrix
subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], offsets, joint_angles)}

# Substitute joint angles into the final transformation matrix
Tfinal_sub = Tfinal.subs(subs_dict)
pprint(Tfinal_sub)

# Extract the end effector position from the transformation matrix
end_effector_position = Tfinal_sub[:3, 3]
pprint(end_effector_position)
