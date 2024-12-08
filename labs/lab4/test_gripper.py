from time import sleep
from kinematics import *
from utils import *



    


    # arm.send_angles([0, 0, 0, 0, 0, 0], 60)

def test(arm):
    arm.sync_send_angles(HOME, 60)
    sleep(2)
    arm.set_gripper_state(1, 50, 1)
    sleep(2)
    arm.set_gripper_state(0, 5, 1)
    sleep(2)
    arm.set_gripper_state(1, 50, 1)
    sleep(2)
    arm.sync_send_angles(STRAIGHT_UP, 60)

def pick_and_place(arm):
    arm.sync_send_angles(HOME, 60)
    sleep(1)
    arm.set_gripper_state(0, 50, 1)
    sleep(1)
    arm.sync_send_angles([-25.31, -7.11, -10.01, -8.43, 60.55, 13.0], 60)
    # sleep(1)
    arm.sync_send_angles([-30.84, 47.00, -28.3, -36.91, 37.26, 60.38], 60)
    sleep(1)
    arm.set_gripper_state(1, 50, 1)
    sleep(1)
    arm.sync_send_angles([-20.31, -7.11, -15.01, -8.43, 60.55, 13.0], 60)
    # sleep(1)
    arm.sync_send_angles([4.39, 27.94, -8.72, -67.41, 21.79, 74.61], 60)
    # sleep(1)
    arm.sync_send_angles([4.39, 27.94, -4.72, -67.41, 21.79, 74.61], 60)
    sleep(1)
    arm.set_gripper_state(0, 50, 1)
    sleep(1)
    arm.sync_send_angles(STRAIGHT_UP, 60)

def main():
    arm = connect_arm()
    print("Arm connected")
    sleep(2)
    pick_and_place(arm)




if __name__ == '__main__':
    main()
