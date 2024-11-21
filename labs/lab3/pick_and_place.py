from time import sleep
from pymycobot import MyCobotSocket
from kinematics import *



def main():
    ip_addr = "10.19.108.58"
    arm = MyCobotSocket(ip_addr, 9000)
    arm.connect_socket()

    sleep(2)

    q_init = arm.get_angles()
    q_init = [0, 0, 0, 0, 0, 0]

    ik = inverse_kinematics(150,0,224, 0,0,0,q_init)

    arm.send_angles(list(ik), 40)




if __name__ == '__main__':
    main()