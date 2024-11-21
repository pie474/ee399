from pymycobot import MechArm, PI_PORT, PI_BAUD, MyCobotSocket
from time import sleep
import numpy as np
import csv

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]
HOME = [0, 0, 0, 0, 0, 0]


CENTER = [-4.74, 26.27, -64.16, 13.79, 90.52, 75.84]
all_angles = [
    CENTER,
    [-9.84, 73.74, -88.5, 65.21, 76.64, 85.25],[],
    CENTER,
    [-12.12, 71.71, -88.5, 66.35, 88.5, 88.85],[],
    CENTER,
    [-4.83, 41.04, -1.58, 99.93, 48.86, 67.67],[],
    CENTER,
    [-9.22, 78.75, -88.5, 61.78, 80.33, 83.32],[],
    CENTER,
    [-12.48, 79.45, -88.5, 64.86, 93.51, 76.55],[],
    CENTER,

    [9.84, 72.68, -64.16, -75.41, 75.32, 56.68],[],
    CENTER,
    [14.32, 79.36, -63.45, -81.21, 86.66, 60.38],[],
    CENTER,
    [16.52, 77.6, -63.36, -77.51, 97.38, 52.64],[],
    CENTER,
    [10.45, 85.25, -63.63, -79.36, 76.72, 62.13],[],
    CENTER,
    [12.3, 86.39, -63.28, -74.09, 84.19, 61.17],[],
    CENTER,

    [6.5, 32.87, -63.45, 5.0, 57.48, 95.62],
    [6.5, 46.84, -63.98, 7.2, 36.56, 91.75],[],
    [6.5, 32.87, -63.45, 5.0, 57.48, 95.62],

    [5.8, 31.64, -63.36, -3.25, 58.71, 98.87],
    [5.8, 44.91, -63.36, -5.18, 36.91, 100.54],[],
    [5.8, 31.64, -63.36, -3.25, 58.71, 98.87],

    [5.71, 32.78, -63.36, -10.63, 57.04, 103.35],
    [2.81, 46.14, -63.36, -11.16, 37.17, 103.79],[],
    [5.71, 32.78, -63.36, -10.63, 57.04, 103.35],

    [2.72, 34.8, -63.28, -14.94, 55.1, 105.55],
    [-1.14, 47.98, -63.45, -17.75, 33.83, 109.86],[],
    [2.72, 34.8, -63.28, -14.94, 55.1, 105.55],

    [-1.31, 31.64, -63.36, -16.08, 60.46, 104.06],
    [-5.53, 48.77, -63.45, -19.68, 32.08, 110.12],[],
    [-1.31, 31.64, -63.36, -16.08, 60.46, 104.06]
]


actual_points = []

def follow_angle_path(arm, angles, speed=30, sleep_time=2):
    for pose in angles:
        if not pose:
            print('Logging point...')
            angles_res = None
            while not angles_res:
                try:
                    angles_res = arm.get_angles()
                except:
                    print('retrying angle getter')
            
            coords_res = None
            while not coords_res:
                try:
                    coords_res = arm.get_coords()
                except:
                    print('retrying coord getter')
                

            actual_points.append((angles_res, coords_res))
            continue
        
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

    sleep(2)

    with open('point_measurements.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'x', 'y', 'z', 'r1', 'r2', 'r3'])
        for angles, pos in actual_points:
            writer.writerow(angles + pos)




if __name__ == '__main__':
    main()