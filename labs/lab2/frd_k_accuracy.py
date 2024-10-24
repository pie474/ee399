import sympy as sp
from kinematics import *
import numpy as np

data = np.genfromtxt('labs/lab2/point_measurements.csv', delimiter=',')[1:,:]


for row in data:
    joint_angles = sp.Matrix(np.radians(row[:6]))
    fk = compute_forward_matrix(joint_angles)
    print((get_translation(fk) - sp.Matrix(row[6:9])).norm())
    # print(sp.N((get_ypr(fk) - sp.Matrix(row[9:12])).norm()))
    # print(sp.N(get_ypr(fk)*180/sp.pi), row[9:12][::-1] + np.array([180, 0, 0]))