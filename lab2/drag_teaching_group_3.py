from pymycobot import MechArm, PI_PORT, PI_BAUD
from time import sleep
import numpy as np

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]
HOME = [0, 0, 0, 0, 0, 0]

anglesXY = np.array([ # XY Plane
    [32.87, 53.34, 16.52, -11.68, 20.03, -0.87],
    [50.44, 73.65, -31.11, -1.75, 49.3, -0.87],
    [39.37, 87.09, -58.27, -6.76, 59.94, -0.87],
    [18.89, 62.66, -6.76, -13.18, 34.89, -0.87],
    [29.88, 81.73, -44.91, -12.12, 44.29, -0.87]
])

anglesXZ = np.array([ # XZ Plane
    [46.43, 46.84, -4.83, 116.27, 57.3, -1.05],
    [37.08, 55.1, -29.7, 107.05, 57.12, -1.05],
    [44.45, 76.02, -25.4, 117.5, 61.17, -1.05],
    [37.79, 70.48, -32.25, 113.02, 60.64, -1.05],
    [37.52, 77.51, -46.93, 112.41, 45.17, -1.05]
])

anglesYZ_pre = np.array([
    [-5.8, 77.87, -22.67, -2.81, -55.72, -0.35],
    [-4.57, 51.24, -18.63, -4.39, -35.59, -0.35],
    [20.03, 77.95, -33.22, 14.23, -46.66, -0.43],
    [17.13, 55.37, -21.79, 11.86, -36.47, -0.43],
    [19.14, 59.32, -16.61, 10.63, -43.15, -0.43]
])

anglesYZ_hit = np.array([
    [-5.09, 82.88, -41.3, -5.53, -44.03, -0.43],
    [-4.57, 56.42, -27-59, -5.8, -31.2, -0.35],
    [16.87, 88.41, -56.25, 10.98, -33.22, -0.43],
    [15.46, 65.12, -41.74, 7.73, -25.13, -0.43],
    [8.52, 62.22, -29.53, 9-14, -30.14, -0.43]
])

anglesYZ = np.array([[pre, hit] for pre, hit in zip(anglesYZ_pre, anglesYZ_hit)]).reshape(10, 6)

def solve_ik(pos, rot, curr_pose):
    ...


def follow_angle_path(arm, angles, speed=30, sleep_time=2):
    for pose in angles:
        arm.send_angles(pose, speed)
        sleep(sleep_time)




def main():
    arm = MechArm(PI_PORT, PI_BAUD)
    arm.power_on()

    arm.send_angles(HOME, 30)
    sleep(2)

    follow_angle_path(arm, anglesXY)

    sleep(2)

    follow_angle_path(arm, anglesXZ)

    sleep(2)

    follow_angle_path(arm, anglesYZ)

    sleep(2)


    arm.send_angles(HOME, 30)

    angles = arm.get_angles()
    coords = arm.get_coords()

    print(angles, coords)


if __name__ == '__main__':
    main()