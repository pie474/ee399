from time import sleep
from pymycobot import MyCobotSocket, MechArmSocket
from kinematics import *
from numpy import pi, atan2, radians, degrees
from sympy import N

HOME = [0,0,0,0,0,0]
STRAIGHT_UP = [0,0,-90,0,0,0]

def connect_arm(ip='10.19.77.243', type = 0):
    if type: 
        arm = MyCobotSocket(ip, 9000)
        arm.connect_socket()
    else: 
        arm = MechArmSocket(ip)
    sleep(0.5)
    return arm

def get_angles(arm):
    angles = None
    while not angles or angles == -1:
        try:
            angles = arm.get_angles()
        except:
            print('retrying angle getter')
    return angles

def get_coords(arm):
    coords = None
    while not coords or coords == -1:
        try:
            coords = arm.get_coords()
        except:
            print('retrying coords getter')
    return coords

def go_to(arm, pos, speed=60, pitch=0, timeout=5):
    """Given a world frame position, compute the inverse kinematics and go to the resulting 
        joint angles synchronously. Roll is locked to 0, and yaw is locked to pointing away from the robot.
    """
    q_init = get_angles(arm)
    rot = get_grab_orientation(pos, pitch)
    print(f'Going to {pos, rot}')
    ik = inverse_kinematics(*pos, *rot, q_init, debug=True).tolist()
    arm.sync_send_angles(ik, speed, timeout=timeout)
    # arm.send_angles(ik, speed)
    # sleep(timeout)
    fk = compute_forward_matrix(radians(get_angles(arm)))
    actual_pos = get_translation(fk).flat()
    actual_rot = N(get_rpy(fk)*180/pi).flat()
    print(f'Actually reached {actual_pos, actual_rot}')

def get_grab_orientation(pos, pitch):
    return [0, pitch+90, degrees(atan2(pos[1], pos[0]))]

def open(arm):
    """Open the arm's gripper."""
    arm.set_gripper_state(0, 50, 1)
    sleep(0.1)

def close(arm):
    """Close the arm's gripper."""
    arm.set_gripper_state(1, 50, 1)
    sleep(0.1)

def offset(pos, offset):
    """Stupid method to add the elements of two position arrays 
    (can be avoided by using numpy arrays for positions)
    """
    assert len(pos) == 3
    return [x + o for x, o, in zip(pos, offset)]

def calibrate_pos(arm, pos, z_offset = 100):
    """Go to a reference point some offset above the given point. 
    This was originally intended to allow the user to provide feedback on the true location of the given point,
        but simply adjusting the prop physically proved to be easier.
    """
    offset_pos = offset(pos, [0, 0, z_offset])
    go_to(arm, offset_pos)
    input(f'waiting at position {offset_pos}')