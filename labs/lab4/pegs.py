from time import sleep
from kinematics import *
from utils import *
from numpy import sqrt
from sympy import Matrix, rot_ccw_axis3, atan2

TRAY_BL = [210, 50, 0]
TRAY_BR = [210, -50, 0]
PICKUP_LOC = [0, -150, 0]
MIDPOINT_LOC = [150, 0, 150]

GRAB_PITCH = 0


_HOLE_SPACING = 50

_HOLE_0 = [30, 30]

HOLES_TRAY_FRAME = [
    Matrix([_HOLE_0[0], _HOLE_0[1], 0, 1]),
    Matrix([_HOLE_0[0] + 0.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1]),
    Matrix([_HOLE_0[0] +       _HOLE_SPACING, _HOLE_0[1], 0]),
    Matrix([_HOLE_0[0] + 1.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1]),
    Matrix([_HOLE_0[0] + 2   * _HOLE_SPACING, _HOLE_0[1], 0]),
    Matrix([_HOLE_0[0] + 2.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1])
]

def get_world_tray_transform(bl: Matrix, br: Matrix):
    dir = br-bl
    ang = atan2(dir[1], dir[0])
    return Matrix([Matrix([[rot_ccw_axis3(ang), bl]]), Matrix([[0, 0, 0, 1]])])

def get_hole_pos_wf(index: int):
    return get_world_tray_transform(Matrix(TRAY_BL + [1]), Matrix(TRAY_BR + [1])) * HOLES_TRAY_FRAME[index]

def go_to(arm, pos, timeout=5):
    q_init = get_angles()
    ik = inverse_kinematics(*pos, q_init).tolist()
    arm.sync_send_angles(ik, 40, timeout=timeout)

def get_grab_oriented_pos(pos, pitch=GRAB_PITCH):
    return pos + [0, pitch, atan2(pos[1], pos[0])]

def open(arm):
    arm.set_gripper_state(0)
    sleep(0.5)

def close(arm):
    arm.set_gripper_state(1)
    sleep(0.5)

def calibrate_pos(arm, pos, z_offset = 50):
    offset_pos = pos
    offset_pos[2] = pos[2] + z_offset
    go_to(arm, offset_pos)
    input(f'waiting at position {pos}')

def get_peg(arm):
    open(arm)
    go_to(arm, get_grab_oriented_pos(PICKUP_LOC))
    close(arm)
    go_to(arm, get_grab_oriented_pos(MIDPOINT_LOC))



def main():
    arm = connect_arm()

    # calibrate_pos(arm, TRAY_BL)
    # calibrate_pos(arm, TRAY_BR)
    calibrate_pos(arm, PICKUP_LOC)

    print('calibration complete')
    sleep(1)

    get_peg(arm)






if __name__ == '__main__':
    main()