from time import sleep
from kinematics import *
from utils import *

PICKUP_LOC = [175, -175, -5]
TOWER_BASE_LOC = [200, 0, -5]
MIDPOINT_LOC = [200, -100, 150]

CUBE_SIZE = 50
HOME = [0,0,0,0,0,0]
STRAIGHT_UP = [0,0,-90,0,0,0]

PRE_1 = [80.5, 4.13, -56.95, -101.77, 62.05, 46.14]
PRE_12 = [74.88, -1.58, -48.42, -88.15, 71.27, 44.03]
GRAB_1 = [74.44, -1.49, -48.33, -76.72, 81.21, 43.5]

PRE_PLACE_1 = [-0.43, 8.96, -90.26, -30.32, 84.99, -0.96]
PRE_PLACE_12 = [-23.46, 20.03, -39.72, 2.81, 82.52, -14.85]
PLACE_1 = [-19.68, 63.19, -68.2, -0.35, 90.0, -14.76]


PRE_2 = [79.1, -27.33, 14.76, 66.09, -76.72, -93.07]
PRE_22 = [79.1, -28.3, 29.09, 71.01, -81.82, -80.41]
GRAB_2 = [78.39, -27.15, 39.02, 74.97, -84.9, -69.96]

PRE_PLACE_2 = [-0.17, -43.15, -1.31, -32.6, 41.3, 20.3]
PRE_PLACE_22 = [-15.55, 0.08, -18.01, -4.74, 66.7, -7.73]
PLACE_2 =[-14.94, 39.02, -68.64, -3.33, 97.99, -11.86]


PRE_3 = [56.6, -8.7, -22.14, -44.2, 81.82, 45.87]
PRE_32 =[50.8, -10.19, -10.89, -39.02, 74.26, 49.3]
GRAB_3 = [50.44, -12.56, 1.93, -41.04, 67.5, 54.75]

PRE_PLACE_3 =[-51.59, 3.16, -99.05, 41.04, 95.53, -11.33]
PRE_PLACE_32 = [-52.11, 5.27, -43.15, 50.88, 78.57, -37.61]
PLACE_3 = [-51.76, 2.81, -44.03, 35.77, 92.19, -40.42]


PRE_4 = [25.83, 25.13, -40.34, -9.31, 85.51, 24.08]
PRE_42 = [25.57, 28.38, -40.34, -8.52, 85.95, 27.33]
GRAB_4 = [25.22, 31.37, -40.34, -7.91, 84.46, 26.1]

PRE_PLACE_4 = [-70.31, -6.5, -42.01, 77.25, 68.55, -42.18]
PRE_PLACE_42 = [-87.89, -30.93, 13.71, 98.34, 79.54, -76.64]
PLACE_4 = [-88.41, -32.43, 19.95, 99.4, 80.77, -79.45]

 
PRE_5 = [21.53, 26.71, -20.47, -5.53, 63.89, 20.3]
PRE_52 = [20.83, 40.86, -19.95, -4.39, 57.21, 24.6]
GRAB_5 = [20.83, 45.7, -19.95, -5.44, 52.29, 24.96]

PRE_PLACE_5 = [-85.34, -7.2, -40.78, 103.97, 64.59, -49.39]
PRE_PLACE_52 = [-85.69, -7.2, -40.78, 95.88, 72.94, -48.42]
PLACE_5 = [-85.42, -5.18, -45.96, 91.31, 77.16, -42.89]

# [74.88, -1.58, -48.42, -88.15, 71.27, 44.03]

# [74.44, -1.49, -48.33, -76.72, 81.21, 43.5]

# [74.44, -1.49, -48.33, -91.58, 66.26, 47.19]

# [4.21, -16.69, -48.42, 1.23, -8.17, 1.84]





def test(arm):
    open(arm)
    go_to(arm, offset(PICKUP_LOC, [0, 0, 100]))
    go_to(arm, PICKUP_LOC, speed=30)
    close(arm)
    go_to(arm, MIDPOINT_LOC)


def send_angles(arm, angles):
    arm.sync_send_angles(angles, 50, timeout=5)

def get_cube(arm, grab, pre, pre2):
    print(f"getting cube {grab}")
    open(arm)
    send_angles(arm, STRAIGHT_UP)
    send_angles(arm, pre)
    send_angles(arm, pre2)
    send_angles(arm, grab)
    close(arm)
    sleep(1)
    send_angles(arm, pre2)
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

def place_cube(arm, place, pre1, pre2):
    print("placing cube")
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
    print("WAITING FOR INPUT")
    input()
    print("CUBE1")
    get_cube(arm, GRAB_1, PRE_1, PRE_12)
    place_cube(arm, PLACE_1, PRE_PLACE_1, PRE_PLACE_12)
    sleep(5)
    print("WAITING FOR INPUT")
    input()
    print("CUBE2")
    get_cube(arm, GRAB_2, PRE_2, PRE_22)
    place_cube(arm, PLACE_2, PRE_PLACE_2, PRE_PLACE_22)
    sleep(5)
    print("WAITING FOR INPUT")
    input()
    print("CUBE3")
    get_cube(arm, GRAB_3, PRE_3, PRE_32)
    place_cube(arm, PLACE_3, PRE_PLACE_3, PRE_PLACE_32)
    sleep(5)
    print("WAITING FOR INPUT")
    input()
    print("CUBE4")
    get_cube(arm, GRAB_4, PRE_4, PRE_42)
    place_cube(arm, PLACE_4, PRE_PLACE_4, PRE_PLACE_42)
    sleep(5)
    print("WAITING FOR INPUT")
    input()
    print("CUBE5")
    get_cube(arm, GRAB_5, PRE_5, PRE_52)
    place_cube(arm, PLACE_5, PRE_PLACE_5, PRE_PLACE_52)
    learn_and_run(arm)



if __name__ == '__main__':
    main()
