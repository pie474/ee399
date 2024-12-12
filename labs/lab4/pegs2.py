from time import sleep
from kinematics import *
from utils import *

PICKUP_LOC = [175, -175, -5]
TOWER_BASE_LOC = [200, 0, -5]
MIDPOINT_LOC = [200, -100, 150]

CUBE_SIZE = 50
HOME = [0,0,0,0,0,0]
STRAIGHT_UP = [0,0,-90,0,0,0]


PRE_GRAB = [-90.96, 55.37, -42.89, -1.4, 56.77, 6.5]

GRAB = [-90.79, 61.69, -42.89, -2.28, 51.5, 7.29]
PRE_PLACE = [-0.35, -43.41, 16.25, 6.32, 66.7, -2.19]

PRE_RED = [3.86, 5.62, -6.41, 1.84, 64.24, 4.65]

PLACE_RED = [3.86, 10.63, -6.41, 1.75, 62.80, 7.03]





def test(arm):
    open(arm)
    go_to(arm, offset(PICKUP_LOC, [0, 0, 100]))
    go_to(arm, PICKUP_LOC, speed=30)
    close(arm)
    go_to(arm, MIDPOINT_LOC)


def send_angles(arm, angles):
    arm.sync_send_angles(angles, 50, timeout=5)

def get_peg(arm, grab, pre):
    print(f"getting cube {grab}")
    open(arm)
    send_angles(arm, STRAIGHT_UP)
    send_angles(arm, pre)
    send_angles(arm, grab)
    close(arm)
    sleep(1)
    send_angles(arm, pre)
    send_angles(arm, STRAIGHT_UP)

def learn_points(arm):
    print("releasing servos")
    sleep(2)
    arm.release_all_servos()
    close(arm)
    sleep(1)
    inp = input()
    # while 1:
    #     sleep(1)
    angles = []
    while not inp:
        a = (arm.get_angles())  # , arm.get_coords())
        print(a)
        angles.append(a)
        inp = input()
    # print("grabbing peg")
    sleep(1)
    # grab_peg(arm)
    return angles

def learn_and_run(arm): 

    # send_angles(arm, STRAIGHT_UP)
    sleep(1)
    angles = learn_points(arm)
    print("STARTING LEARNED ROUTINE")
    curr = get_angles(arm)
    send_angles(arm, curr)
    send_angles(arm, STRAIGHT_UP)
    for a in angles:
        arm.sync_send_angles(a, 60, timeout = 5)
    
    send_angles(arm, STRAIGHT_UP)
    sleep(1)

def place_peg(arm, place, pre1, pre2):
    print("placing peg")
    send_angles(arm, STRAIGHT_UP)
    send_angles(arm, pre1)
    send_angles(arm, pre2)
    send_angles(arm, place)
    open(arm)
    sleep(1)
    send_angles(arm, pre2)
    send_angles(arm, pre1)
    send_angles(arm, STRAIGHT_UP)


def main():
    arm = connect_arm()
    print("arm connected")
    get_peg(arm, GRAB, PRE_GRAB)
    place_peg(arm, PLACE_RED, PRE_PLACE, PRE_RED)
    learn_and_run(arm)



if __name__ == '__main__':
    main()
