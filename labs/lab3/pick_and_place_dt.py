from pymycobot import MyCobotSocket
from time import sleep

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]
HOME = [0, 0, 0, 0, 0, 0]


CENTER = [-4.74, 26.27, -64.16, 13.79, 90.52, 75.84]
all_angles = [
    [-27.5, 119.88, -90.52, -7.64, 49.13, -34.54],
    [-5.44, 81.91, -90.52, 6.32, 78.75, -31.11],
    [17.75, 119.53, -90.43, 5.97, 56.42, -31.11]    
]

all_angles_elevated = [
    [-16.61, 74.53, -58.0, -13.71, 63.36, 2.02],
    [-2.37, 48.07, -58.0, -1.31, 79.8, -0.43],
    [25.75, 74.44, -58.0, -0.61, 69.43, 5.71]
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
        follow_angle_path(arm, all_angles_elevated, speed=50, sleep_time=1)

        # actual_points.append(([], []))

    arm.send_angles(HOME, 30)




if __name__ == '__main__':
    main()
