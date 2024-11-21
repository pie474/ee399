from time import sleep
from pymycobot import MyCobotSocket
from kinematics import *

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]

def get_angles(arm):
    angles = None
    while not angles or angles == -1:
        try:
            angles = arm.get_angles()
        except:
            print('retrying angle getter')
    return angles

def main():
    ip_addr = "10.19.108.58"
    arm = MyCobotSocket(ip_addr, 9000)
    arm.connect_socket()

    sleep(2)

    arm.sync_send_angles(STRAIGHT_UP, 60)

    sleep(2)

    q_init = get_angles(arm)
    q_init = [0, 0, 0, 0, 0, 0]

    sleep(2)

    ik = inverse_kinematics(150,0,224, 0,0,0,q_init).tolist()

    arm.send_angles(ik, 40)

    # arm.send_angles([0, 0, 0, 0, 0, 0], 60)





if __name__ == '__main__':
    main()
