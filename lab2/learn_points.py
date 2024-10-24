from pymycobot import MechArm, PI_PORT, PI_BAUD
from time import sleep

def main():
    arm = MechArm(PI_PORT, PI_BAUD)
    arm.power_on()

    sleep(1)
    arm.release_all_servos()
    sleep(1)

    inp = input()

    while not inp:
        print(arm.get_angles())  # , arm.get_coords())
        inp = input()

    arm.send_angles([0, 0, 0, 0, 0, 0])
    sleep(1)


if __name__ == '__main__':
    main()