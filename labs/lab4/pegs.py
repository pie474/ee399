from time import sleep
from kinematics import *
from utils import *
import numpy as np
from numpy import sqrt, array
from sympy import Matrix, rot_ccw_axis3, atan2

# Origin of the tray frame, defined to be the bottom left of the plate with the holes
#   (where it intersects with the legs)
TRAY_BL = [260, 70, 30]

# Location of the botom right of the plate. Only the angle from TRAY_BL is used to 
#   compute tray orientation.
TRAY_BR = [260, -50, 30]

PICKUP_LOC = [175, -175, 20]  # Location where pegs are placed by the user for the arm to pick up
MIDPOINT_LOC = [200, -100, 150]  # Location to return to between pickup/place operations to avoid collisions

GRAB_PITCH = 45  # EE pitch to use when grabbing/placing pegs. (NOTE: doesn't seem to affect IK)

# Specify the type of tray we're using:
# 0 - the parallelogram 6 hole one
# 1 - the rectangular 8 hole one
TRAY_TYPE = 1 

if TRAY_TYPE == 0:
    _HOLE_SPACING = 50  # distance between holes
    _HOLE_0 = [30, 30]  # position of the bottom left hole in tray frame

    HOLES_TRAY_FRAME = [
        Matrix([_HOLE_0[0]                      , _HOLE_0[1]                          , 0, 1]),
        Matrix([_HOLE_0[0] + 0.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1]),
        Matrix([_HOLE_0[0] +       _HOLE_SPACING, _HOLE_0[1]                          , 0, 1]),
        Matrix([_HOLE_0[0] + 1.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1]),
        Matrix([_HOLE_0[0] + 2   * _HOLE_SPACING, _HOLE_0[1]                          , 0, 1]),
        Matrix([_HOLE_0[0] + 2.5 * _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING*sqrt(3)/2, 0, 1])
    ]
elif TRAY_TYPE == 1:
    _HOLE_SPACING = 50  # distance between holes
    _HOLE_0 = [75, 25]  # position of the bottom left hole in tray frame

    HOLES_TRAY_FRAME = [
        Matrix([_HOLE_0[0]                , _HOLE_0[1] + _HOLE_SPACING, 0, 1]),
        Matrix([_HOLE_0[0] + _HOLE_SPACING, _HOLE_0[1] + _HOLE_SPACING, 0, 1]),
        Matrix([_HOLE_0[0]                , _HOLE_0[1]                , 0, 1]),
        Matrix([_HOLE_0[0] + _HOLE_SPACING, _HOLE_0[1]                , 0, 1])
    ]

NUM_PEGS = len(HOLES_TRAY_FRAME)

def get_world_tray_transform(bl: Matrix, br: Matrix):
    """Compute the transformation matrix from world frame to tray frame using the corner positions."""
    dir = br-bl
    ang = atan2(dir[1], dir[0])
    return Matrix([Matrix([[rot_ccw_axis3(ang), bl]]), Matrix([[0, 0, 0, 1]])])

def get_hole_pos_wf(index: int):
    """Return the world frame position of the hole at the given index."""
    res = get_world_tray_transform(Matrix(TRAY_BL), Matrix(TRAY_BR)) * HOLES_TRAY_FRAME[index]
    return list(map(float, res.flat()))[:3]

def get_peg(arm):
    """Pick up a peg from the pickup location."""
    open(arm)
    go_to(arm, offset(PICKUP_LOC, [0, 0, 100]))
    go_to(arm, PICKUP_LOC, pitch=GRAB_PITCH)
    close(arm)
    go_to(arm, MIDPOINT_LOC)

def place_peg(arm, index):
    """Assuming a peg is in the gripper, place it in the hole specified by the index."""
    hole_pos = get_hole_pos_wf(index)
    go_to(arm, offset(hole_pos, [0, 0, 90]), pitch=GRAB_PITCH)
    go_to(arm, offset(hole_pos, [0, 0, 50]), pitch=GRAB_PITCH)
    open(arm)
    go_to(arm, MIDPOINT_LOC)

def peg_routine(arm):
    """Pick up and place all pegs into their respective holes."""
    for i in range(NUM_PEGS):
        get_peg(arm)
        place_peg(arm, i)

def hammer_routine(arm):
    """Pick up the hammer and push all the pegs in.
    Note: Rectangular tray seems like it would need too much force, so this does nothing 
            in that case.
    """
    if TRAY_TYPE == 1:
        return

    get_peg(arm)
    for i in range(NUM_PEGS):
        hole_pos = get_hole_pos_wf(i)
        go_to(arm, offset(hole_pos, [0, 0, 90]), pitch=GRAB_PITCH)
        go_to(arm, offset(hole_pos, [0, 0, 50]), pitch=GRAB_PITCH)
        go_to(arm, offset(hole_pos, [0, 0, 90]), pitch=GRAB_PITCH)

def main():
    arm = connect_arm()

    # set_tool(T_GRIPPER)


    calibrate_pos(arm, TRAY_BL, z_offset=150)
    # calibrate_pos(arm, TRAY_BR)
    calibrate_pos(arm, PICKUP_LOC, z_offset=100)

    for i in range(NUM_PEGS):
        calibrate_pos(arm, get_hole_pos_wf(i), z_offset=100)

    print('\n================  CALIBRATION COMPLETE  ================\n')
    
    sleep(1)

    peg_routine(arm)
    hammer_routine(arm)



if __name__ == '__main__':
    main()