from time import sleep
from utils import *

def main():
    ip_addr = "10.19.108.58"
    # arm = MechArmSocket(ip_addr, debug=True)
    arm = connect_arm()
    res = get_angles(arm)
    print(res)
    sleep(1)
    # arm.sync_send_angles(HOME, 20, timeout=2)
    # sleep(2)

    # # grab_pen(arm)
    
    # # arm.sync_send_angles([10,0,0,0,0,0], 20, timeout=2)
    # sleep(2)
    # arm.sync_send_angles(STRAIGHT_UP, 40, timeout=5)
    # sleep(2)
    # test_gripper(arm)
    # print("gripper open")
    # arm.set_gripper_state(0, 60)
    # sleep(3)
    
    # arm.set_gripper_value(50,60, gripper_type = 3)
    # # arm = MechArm(PI_PORT, PI_BAUD)
    # # arm.power_on()
    sleep(2)
    print("releasing servos")
    sleep(2)
    arm.release_all_servos()
    sleep(1)

    inp = input()
    # while 1:
    #     sleep(1)
    while not inp:
        print(arm.get_angles())  # , arm.get_coords())
        inp = input()

    arm.send_angles([0, 0, 0, 0, 0, 0], 50)
    sleep(1)

def test_gripper(arm):
    for i in range (2):
        print('gripper 0')
        arm.set_gripper_state(0, 60)
        sleep(2)
        print('gripper 1')
        arm.set_gripper_state(1, 90)
        sleep(2)
        print('try get value 3')
        arm.set_gripper_value(50,60, gripper_type = 3)
        sleep(2)
        print('try get value 4')
        arm.set_gripper_value(50,60, gripper_type = 4)
        sleep(2)
        print('try get value 1')
        arm.set_gripper_value(50,60, gripper_type = 1)
        sleep(2)

def grab_pen(arm):
    arm.set_gripper_state(254, 50)
    #sleep(2)
    #arm.set_gripper_state(0, 50)

if __name__ == '__main__':
    main()