import sympy as sp
from ..kinematics import *

x, y, z = sp.symbols('x, y, z')

x = 1 + z*2
y = -1 + z*3

a, b, c = sp.symbols('a, b, c')

r03 = sp.rot_axis3(a) * sp.rot_axis2(b) * sp.rot_axis1(c)
# sp.pprint(r03)

r03d = sp.eye(3)

sol = sp.solve(r03 - r03d, (a, b, c))

sp.pprint(sol)