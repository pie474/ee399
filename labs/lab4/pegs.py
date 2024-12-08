from struct import iter_unpack
from time import sleep
from kinematics import *
from utils import *
import numpy as np
from numpy import sqrt
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

def go_to(arm, pos, pitch=GRAB_PITCH, timeout=5):
    q_init = get_angles(arm)
    rot = get_grab_orientation(pos, pitch)
    print(f'Going to {pos, rot}')
    ik = inverse_kinematics(*pos, *rot, q_init, debug=True).tolist()
    arm.sync_send_angles(ik, 60, timeout=timeout)
    # arm.send_angles(ik, 60)
    # sleep(timeout)
    fk = compute_forward_matrix(np.radians(get_angles(arm)))
    actual_pos = get_translation(fk).flat()
    actual_rot = N(get_rpy(fk)*180/np.pi).flat()
    print(f'Actually reached {actual_pos, actual_rot}')

def get_grab_orientation(pos, pitch):
    return [0, pitch+90, np.degrees(np.atan2(pos[1], pos[0]))]

def open(arm):
    arm.set_gripper_state(0, 50, 1)
    sleep(0.1)

def close(arm):
    arm.set_gripper_state(1, 50, 1)
    sleep(0.1)

def offset(pos, offset):
    assert len(pos) == 3
    return [x + o for x, o, in zip(pos, offset)]

def calibrate_pos(arm, pos, z_offset = 100):
    offset_pos = offset(pos, [0, 0, z_offset])
    go_to(arm, offset_pos)
    input(f'waiting at position {offset_pos}')

def get_peg(arm):
    open(arm)
    sleep(0.5)
    go_to(arm, offset(PICKUP_LOC, [0, 0, 100]))
    go_to(arm, PICKUP_LOC)
    sleep(0.5)
    close(arm)
    sleep(0.5)
    go_to(arm, MIDPOINT_LOC, pitch=0)
    sleep(0.5)

def deposit_peg(arm, index):
    hole_pos = get_hole_pos_wf(index)
    go_to(arm, offset(hole_pos, [0, 0, 90]))
    go_to(arm, offset(hole_pos, [0, 0, 50]))
    open(arm)
    go_to(arm, MIDPOINT_LOC, pitch=0)

def peg_routine(arm):
    for i in range(6):
        get_peg(arm)
        deposit_peg(arm, i)

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