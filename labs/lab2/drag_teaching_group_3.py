from pymycobot import MechArm, PI_PORT, PI_BAUD
from time import sleep
import numpy as np
import csv

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]
HOME = [0, 0, 0, 0, 0, 0]

angles_xy_yz = [
    [3.25, 55.37, -1.58, -8.34, 31.2, -0.26],[],
    HOME,
    [36.21, 55.45, -1.58, 3.07, 29.97, -0.35],[],
    HOME,
    [32.25, 81.21, -55.63, 0.08, 54.58, -0.35],[],
    HOME,
    [17.22, 82.08, -55.72, -4.48, 51.06, -0.35],[],
    HOME,
    [2.02, 82.0, -55.63, -7.55, 52.55, -0.35],[],
    HOME,
    [23.02, 69.25, -55.63, 95.71, 60.46, -0.35],[],
    HOME,
    [27.07, 44.56, -42.01, 100.37, 53.7, -0.35],[],
    HOME,
    [28.12, 79.71, -39.63, 113.81, 53.61, -0.35],[],
    HOME,
    [33.66, 37.61, 0.26, 112.76, 55.54, -0.35],[],
    HOME,
    [38.4, 39.9, 11.42, 122.25, 57.74, -0.35],[],
]

angles_xz = [
    [25.83, 40.78, 17.75, 13.27, -37.35, -0.26],
    [21.26, 70.66, -36.21, 9.05, -18.63, -0.26],[],
    [25.83, 40.78, 17.75, 13.27, -37.35, -0.26],
    # HOME,
    [14.58, 33.22, 30.32, 12.04, -43.15, -0.26],
    [12.91, 60.29, -19.07, 11.51, -27.68, -0.26],[],
    [14.58, 33.22, 30.32, 12.04, -43.15, -0.26],
    # HOME,
    [-2.37, 32.6, 32.16, 13.09, -43.24, -0.26],
    [-6.76, 59.23, -16.69, 7.91, -28.12, -0.26],[],
    [-2.37, 32.6, 32.16, 13.09, -43.24, -0.26],
    # HOME,
    [-6.67, -2.72, -7.2, 11.16, 24.78, -0.26],
    [-3.33, 32.87, -37.44, 9.58, 14.94, -0.26],[],
    [-6.67, -2.72, -7.2, 11.16, 24.78, -0.26],
    # HOME,
    [20.21, 5.53, 10.81, 5.36, -9.66, -0.26],
    [17.84, 42.36, -42.09, 5.44, 4.92, -0.26],[],
    [20.21, 5.53, 10.81, 5.36, -9.66, -0.26],
]

angles_xz_2 = [
    HOME,
    [2.9, 19.42, -41.39, 14.06, 48.86, -0.26],
    [4.57, 35.06, -42.01, 12.91, 23.73, -0.26],[],
    [2.9, 19.42, -41.39, 14.06, 48.86, -0.26],
    HOME,
    [-6.06, 8.61, -21.88, 14.15, 36.38, -0.26],
    [-5.27, 37.08, -45.26, 15.2, 25.22, -0.26],[],
    [-6.06, 8.61, -21.88, 14.15, 36.38, -0.26],
    HOME,
    [17.22, 4.83, -8.52, 13.44, 25.92, -0.26],
    [17.13, 49.3, -57.56, 11.16, 28.74, -0.26],[],
    [17.22, 4.83, -8.52, 13.44, 25.92, -0.26],
    HOME,
    [-5.0, 7.03, -9.4, 16.61, 36.21, -0.26],
    [-4.83, 45.0, -45.52, 19.42, 34.8, -0.26],[],
    [-5.0, 7.03, -9.4, 16.61, 36.21, -0.26],
    HOME,
    [8.26, 3.33, -1.05, 18.45, 28.65, -0.26],
    [8.78, 46.93, -49.04, 17.4, 32.95, -0.26],[],
    [8.26, 3.33, -1.05, 18.45, 28.65, -0.26],
    HOME,
]

actual_points = []

def follow_angle_path(arm, angles, speed=30, sleep_time=2):
    for pose in angles:
        if not pose:
            actual_points.append((arm.get_angles(), arm.get_coords()))
            continue
        
        arm.sync_send_angles(pose, speed, timeout=2)
        sleep(sleep_time)




def main():
    arm = MechArm(PI_PORT, PI_BAUD)
    arm.power_on()

    sleep(2)

    arm.sync_send_angles(HOME, 50, timeout=3)
    # arm.release_all_servos()
    sleep(2)

    for i in range(2):
        follow_angle_path(arm, angles_xy_yz, speed=50, sleep_time=0.3)

        sleep(2)

        follow_angle_path(arm, angles_xz_2, speed=50, sleep_time=0.3)

        sleep(2)
        # actual_points.append(([], []))

    arm.send_angles(STRAIGHT_UP, 30)

    sleep(2)

    with open('point_measurements.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'x', 'y', 'z', 'r1', 'r2', 'r3'])
        for angles, pos in actual_points:
            writer.writerow(angles + pos)




if __name__ == '__main__':
    main()