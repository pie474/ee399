from time import sleep
from kinematics import *
from utils import *
import numpy as np
from numpy import sqrt, array
from sympy import Matrix, rot_ccw_axis3, atan2

TRAY_BL = [210, 50, 30]
TRAY_BR = [210, -50, 30]
PICKUP_LOC = [175, -175, 20]
MIDPOINT_LOC = [200, -100, 150]

GRAB_PITCH = 45


_HOLE_SPACING = 50
_HOLE_0 = [30, 30]

HOLES_TRAY_FRAME = [
    Matrix([_HOLE_0[0]                      , _HOLE_0[1]                          , 0, 1]),
    Matrix([_HOLE_0[0] + 0.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1]),
    Matrix([_HOLE_0[0] +       _HOLE_SPACING, _HOLE_0[1]                          , 0, 1]),
    Matrix([_HOLE_0[0] + 1.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1]),
    Matrix([_HOLE_0[0] + 2   * _HOLE_SPACING, _HOLE_0[1]                          , 0, 1]),
    Matrix([_HOLE_0[0] + 2.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1])
]

def get_world_tray_transform(bl: Matrix, br: Matrix):
    dir = br-bl
    ang = atan2(dir[1], dir[0])
    return Matrix([Matrix([[rot_ccw_axis3(ang), bl]]), Matrix([[0, 0, 0, 1]])])

def get_hole_pos_wf(index: int):
    res = get_world_tray_transform(Matrix(TRAY_BL), Matrix(TRAY_BR)) * HOLES_TRAY_FRAME[index]
    return list(map(float, res.flat()))[:3]

def get_peg(arm):
    open(arm)
    go_to(arm, offset(PICKUP_LOC, [0, 0, 100]))
    go_to(arm, PICKUP_LOC, pitch=GRAB_PITCH)
    close(arm)
    go_to(arm, MIDPOINT_LOC)

def place_peg(arm, index):
    hole_pos = get_hole_pos_wf(index)
    go_to(arm, offset(hole_pos, [0, 0, 90]), pitch=GRAB_PITCH)
    go_to(arm, offset(hole_pos, [0, 0, 50]), pitch=GRAB_PITCH)
    open(arm)
    go_to(arm, MIDPOINT_LOC)

def peg_routine(arm):
    for i in range(6):
        get_peg(arm)
        place_peg(arm, i)

def main():
    arm = connect_arm()

    # set_tool(T_GRIPPER)

    # calibrate_pos(arm, TRAY_BL)
    # calibrate_pos(arm, TRAY_BR)
    calibrate_pos(arm, PICKUP_LOC, z_offset=100)

    print('\n================  CALIBRATION COMPLETE  ================\n')
    
    sleep(1)

    peg_routine(arm)

    # for i in range(6):
    #     hole_pos = get_hole_pos_wf(i)
    #     go_to(arm, offset(hole_pos, [0, 0, 50]))

    # go_to(arm, PICKUP_LOC)



if __name__ == '__main__':
    main()