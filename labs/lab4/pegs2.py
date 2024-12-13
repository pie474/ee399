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


PRE_PLACE_2 = [-5.53, 14.67, -28.03, -5.44, 84.28, 38.67]

PRE_ORANGE = [-5.62, 21.79, -24.25, -6.06, 78.04, 36.47]

PLACE_ORANGE = [-5.8, 26.19, -24.16, -5.88, 76.9, 31.2]


PRE_PLACE_3 = [0.43, -2.46, -20.21, 8.87, 91.93, -10.1]

PRE_GREEN = [0.52, -2.37, 3.69, 8.17, 72.5, -10.1]

PLACE_GREEN = [0.52, 3.86, 4.21, 7.73, 71.01, -8.17]


PRE_PLACE_4 = [-0.61, -34.54, 8.17, -4.74, 79.62, 0.79]

PRE_BLUE = [-0.61, -5.27, 8.08, -7.55, 69.16, 7.55]

PLACE_BLUE = [-7.03, 0.87, 8.26, -2.98, 66.79, -2.19]

# PRE_RED_HAMMER
P_R_H = [9.05, 0.79, -40.25, -0.17, 84.37, -1.84]
R_H = [9.49, -10.89, 15.38, -3.07, 45.7, 0.43]

# PRE_ORANGE_HAMMER
P_O_H = [-0.35, -5.8, -31.02, -6.24, 78.22, -0.26]
O_H = [-0.35, -22.41, 31.2, -11.77, 35.5, 7.11]

# PRE_GREEN_HAMMER
P_G_H = [8.61, -30.84, -12.74, 2.37, 82.7, 0.52]
G_H = [0.26, -21.53, 15.98, 7.47, 76.72, -11.86]
# [8.34, -40.07, 36.91, 0.7, 51.85, 0.08]

# PRE_BLUE_HAMMER
P_B_H = [7.99, -23.81, -14.23, -9.22, 85.07, 3.69]
B_H = [7.99, -45.0, 37.96, -16.69, 56.25, 13.0]





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
    # [-10.19, 11.33, -43.41, 0.96, 90.96, -5.0]
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

def hammer_peg(arm, place, pre1):
    print("hammering peg")
    # send_angles(arm, STRAIGHT_UP)
    send_angles(arm, pre1)
    send_angles(arm, place)
    sleep(1)
    send_angles(arm, pre1)
    # send_angles(arm, STRAIGHT_UP)


def main():
    arm = connect_arm()
    print("arm connected")
    get_peg(arm, GRAB, PRE_GRAB)
    place_peg(arm, PLACE_RED, PRE_PLACE, PRE_RED)
    get_peg(arm, GRAB, PRE_GRAB)
    place_peg(arm, PLACE_ORANGE, PRE_PLACE_2, PRE_ORANGE)
    get_peg(arm, GRAB, PRE_GRAB)
    place_peg(arm, PLACE_GREEN, PRE_PLACE_3, PRE_GREEN)
    get_peg(arm, GRAB, PRE_GRAB)
    place_peg(arm, PLACE_BLUE, PRE_PLACE_4, PRE_BLUE)
    get_peg(arm, GRAB, PRE_GRAB)
    hammer_peg(arm, R_H, P_R_H)
    hammer_peg(arm, O_H, P_O_H)
    hammer_peg(arm, G_H, P_G_H)
    hammer_peg(arm, B_H, P_B_H)
    learn_and_run(arm)
    




if __name__ == '__main__':
    main()
