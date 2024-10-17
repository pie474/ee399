from pymycobot import MechArm
from pymycobot import PI_PORT, PI_BAUD
from time import sleep


arm = MechArm(PI_PORT, PI_BAUD)
arm.power_on()

STRAIGHT_UP = [0, 0, -90, 0, 0, 0]
HOME = [0, 0, 0, 0, 0, 0]

posesXY = [ # XY Plane
    [32.87, 53.34, 16.52, -11.68, 20.03, -0.87],
    [50.44, 73.65, -31.11, -1.75, 49.3, -0.87],
    [39.37, 87.09, -58.27, -6.76, 59.94, -0.87],
    [18.89, 62.66, -6.76, -13.18, 34.89, -0.87],
    [29.88, 81.73, -44.91, -12.12, 44.29, -0.87]
]

posesXZ = [ # XZ Plane
    [46.43, 46.84, -4.83, 116.27, 57.3, -1.05],
    [37.08, 55.1, -29.7, 107.05, 57.12, -1.05],
    [44.45, 76.02, -25.4, 117.5, 61.17, -1.05],
    [37.79, 70.48, -32.25, 113.02, 60.64, -1.05],
    [37.52, 77.51, -46.93, 112.41, 45.17, -1.05]
]


def follow_angle_path(poses, speed=30, sleep_time=2):
    for pose in poses:
        arm.send_angles(pose, speed)
        sleep(sleep_time)


arm.send_angles(HOME, 30)
sleep(2)

follow_angle_path(posesXY)

sleep(2)

# solution = 

arm.send_angles(HOME, 30)

angles = arm.get_angles()
coords = arm.get_coords()

print(angles, coords)
