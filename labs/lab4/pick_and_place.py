from time import sleep
from kinematics import *
from utils import *

def main():
    arm = connect_arm()
    print("Arm connected")
    sleep(2)

    arm.sync_send_angles(STRAIGHT_UP, 60)

    sleep(2)
    # print(get_coords(arm))
    # q_init = get_angles(arm)
    q_init = [0, 0, 0, 0, 0, 0]

    sleep(2)
    print("trying IK")
    ik = inverse_kinematics(0,0,374, 0,0,0,q_init).tolist()
    print("sending IK")
    print(ik)
    arm.send_angles(ik, 40)

    # arm.send_angles([0, 0, 0, 0, 0, 0], 60)





if __name__ == '__main__':
    main()
