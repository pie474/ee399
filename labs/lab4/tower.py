from time import sleep
from kinematics import *
from utils import *

PICKUP_LOC = [175, -175, -5]
TOWER_BASE_LOC = [200, 0, -5]
MIDPOINT_LOC = [200, -100, 150]

CUBE_SIZE = 50


def get_cube(arm):
    open(arm)
    go_to(arm, offset(PICKUP_LOC, [0, 0, 100]))
    go_to(arm, PICKUP_LOC)
    close(arm)
    go_to(arm, MIDPOINT_LOC)

def place_cube(arm, index):
    cube_z = index * CUBE_SIZE
    go_to(arm, offset(TOWER_BASE_LOC, [0, 0, cube_z + 90]))
    go_to(arm, offset(TOWER_BASE_LOC, [0, 0, cube_z + 50]))
    open(arm)
    go_to(arm, MIDPOINT_LOC)

def cube_routine(arm, parity):
    indices = range(1 if parity else 0, 6, 2)
    for index in indices:
        input(f'Waiting to place cube {index}')
        get_cube(arm)
        place_cube(arm, index)


def main():
    arm = connect_arm()

    # set_tool(T_GRIPPER)


    calibrate_pos(arm, TOWER_BASE_LOC, z_offset=100)
    calibrate_pos(arm, PICKUP_LOC, z_offset=100)

    print('\n================  CALIBRATION COMPLETE  ================\n')
    
    sleep(1)

    cube_routine(arm)

    # for i in range(6):
    #     hole_pos = get_hole_pos_wf(i)
    #     go_to(arm, offset(hole_pos, [0, 0, 50]))

    # go_to(arm, PICKUP_LOC)



if __name__ == '__main__':
    main()