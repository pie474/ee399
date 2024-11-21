from pymycobot import MechArm, PI_PORT, PI_BAUD, MyCobotSocket
from time import sleep

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]
HOME = [0, 0, 0, 0, 0, 0]


CENTER = [-4.74, 26.27, -64.16, 13.79, 90.52, 75.84]
all_angles = [
    
]

def follow_angle_path(arm, angles, speed=30, sleep_time=2):
    for pose in angles:
        arm.sync_send_angles(pose, speed, timeout=2)
        sleep(sleep_time)


def main():
    ip_addr = "10.19.108.58"
    arm = MyCobotSocket(ip_addr, 9000)
    arm.connect_socket()

    # arm.power_on()

    sleep(2)

    arm.sync_send_angles(HOME, 50, timeout=3)
    # arm.release_all_servos()
    sleep(2)

    for i in range(1):
        follow_angle_path(arm, all_angles, speed=50, sleep_time=0.3)

        # actual_points.append(([], []))

    arm.send_angles(STRAIGHT_UP, 30)




if __name__ == '__main__':
    main()