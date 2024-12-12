from time import sleep
from kinematics import *
from utils import *



ABOVE_PEG = [-58.09, -5.8, 2.02, 2.02, -3.51, 3.95]
AT_PEG = [-58.0, 52.2, 3.86, -2.72, -65.12, -0.7]
HOLE_1 =[33.13, 22.14, 16.43, -5.71, -46.05, 4.04]

HOLE_2 = [23.46, 40.16, -9.05, 3.77, -36.38, -4.48]

HOLE_3 = [23.55, 13.35, 33.83, 6.94, -51.94, -7.91]

# HOLE_4 = [7.11, 33.22, 3.77, -6.06, -41.57, 3.33]

HOLE_5 = [6.15, 7.99, 39.72, 5.36, -50.88, -7.99]

HOLE_6 = [-6.06, 33.92, 1.23, -9.22, -39.99, 3.07]
ABOVE_1 = [37.52, 17.66, 9.93, 13.53, -31.9, -13.79]

ABOVE_2 =[24.87, 32.95, -15.46, 13.18, -23.11, -13.88]

# ABOVE_3 =[24.87, 32.95, -15.46, 13.18, -23.11, -13.88]

ABOVE_3 =[24.96, 8.61, 28.65, 5.62, -42.45, -8.61]

ABOVE_4 = [13.44, 27.33, 4.21, 11.33, -36.91, -12.12]

HOLE_4  = [14.06, 33.57, 10.19, 12.48, -51.32, -9.66]

ABOVE_5 = [6.67, -2.98, 36.82, 9.93, -39.63, -7.38]

ABOVE_6 = [-2.46, 19.51, 6.59, 5.53, -32.95, -5.09]


    # arm.send_angles([0, 0, 0, 0, 0, 0], 60)

def test(arm):
    arm.sync_send_angles(HOME, 60, timeout=4)
    sleep(2)
    arm.set_gripper_state(1, 50, 1)
    sleep(2)
    arm.set_gripper_state(0, 5, 1)
    sleep(2)
    arm.set_gripper_state(1, 50, 1)
    sleep(2)
    arm.sync_send_angles( [-31.577, 17.22, 56.76, 0, 74.91, -5.41], 60, timeout=4)

def pick_and_place(arm):
    print('HOME')
    arm.sync_send_angles(HOME, 60, timeout=5)
    sleep(1)
    print('open')
    arm.set_gripper_state(0, 50, 1)
    sleep(1)
    print('ABOVE')
    arm.sync_send_angles(ABOVE_PEG, 60, timeout=5)
    # sleep(1)
    print('AT')
    arm.sync_send_angles(AT_PEG, 60, timeout=5)
    sleep(1)
    print('close')
    arm.set_gripper_state(1, 50, 1)
    sleep(1)
    print('ABOVE')
    arm.sync_send_angles(ABOVE_PEG, 60, timeout=5)
    # sleep(1)
    print('HOME')
    arm.sync_send_angles(HOME, 60, timeout=5)
    # sleep(1)
    print('ABOVE HOLE')
    arm.sync_send_angles(ABOVE_6, 60, timeout = 5)
    print('HOLE1')
    arm.sync_send_angles(HOLE_6, 60, timeout=5)
    sleep(1)
    print('open')
    arm.set_gripper_state(0, 50, 1)
    sleep(1)
    print('ABOVE HOLE')
    arm.sync_send_angles(ABOVE_6, 60, timeout = 5)
    print('HOME')
    arm.sync_send_angles(HOME, 60)

def grab_peg(arm):
    print('HOME')
    arm.sync_send_angles(HOME, 60, timeout=5)
    sleep(1)
    print('open')
    arm.set_gripper_state(0, 50, 1)
    sleep(1)
    print('ABOVE')
    arm.sync_send_angles(ABOVE_PEG, 60, timeout=5)
    # sleep(1)
    print('AT')
    arm.sync_send_angles(AT_PEG, 60, timeout=5)
    sleep(1)
    print('close')
    arm.set_gripper_state(1, 50, 1)
    sleep(1)
    print('ABOVE')
    arm.sync_send_angles(ABOVE_PEG, 60, timeout=5)
    # sleep(1)
    print('HOME')
    arm.sync_send_angles(HOME, 60, timeout=5)


def above_holes(arm):
    print('HOME')
    arm.sync_send_angles(HOME, 60, timeout=5)
    # sleep(1)
    # print('1')
    # arm.sync_send_angles(ABOVE_1, 60, timeout=5)
    # arm.sync_send_angles(HOLE_1, 60, timeout=5)
    # # sleep(1)
    # print('2')
    # arm.sync_send_angles(ABOVE_2, 60, timeout=5)
    # arm.sync_send_angles(HOLE_2, 60, timeout=5)
    # # sleep(1)
    # print('3')
    # arm.sync_send_angles(ABOVE_3, 60, timeout=5)
    # arm.sync_send_angles(HOLE_3, 60, timeout=5)
    # sleep(1)
    print('4')
    arm.sync_send_angles(ABOVE_4, 60, timeout=5)
    arm.sync_send_angles(HOLE_4, 60, timeout=5)
    # sleep(1)
    print('5')
    arm.sync_send_angles(ABOVE_5, 60, timeout=5)
    arm.sync_send_angles(HOLE_5, 60, timeout=5)
    # sleep(1)
    print('6')
    arm.sync_send_angles(ABOVE_6, 60, timeout=5)
    arm.sync_send_angles(HOLE_6, 60, timeout=5)
    # sleep(1)

def learn_points(arm):
    print("releasing servos")
    sleep(2)
    arm.release_all_servos()
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
    print("grabbing peg")
    sleep(1)
    grab_peg(arm)

    for a in angles:
        arm.sync_send_angles(a, 60, timeout = 5)
    
    arm.send_angles([0, 0, 0, 0, 0, 0], 50)
    sleep(1)

def main():
    arm = connect_arm()
    print("Arm connected")
    sleep(2)
    grab_peg(arm)
    pick_and_place(arm)




if __name__ == '__main__':
    main()
