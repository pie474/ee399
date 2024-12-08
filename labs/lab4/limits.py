from utils import *
import time

arm = connect_arm()

def sync_send_angles(degrees, speed, timeout=15):
    global arm
    t = time.time()
    arm.send_angles(degrees, speed)
    while time.time() - t < timeout:
        f = arm.is_in_position(degrees, 0)
        if f:
            return True
        time.sleep(0.1)
    return False

# arm.send_angles([-45, 120, -84, 0, -75, -180], 50)
a = 0.875
# arm.send_angles([-45, 120*a, -84*a, 0, -75, -0], 50)
arm.send_angles([0, 0, 0, 0, 0, 1000], 50)
# 165 90 (-180, 70) 165 115 175

# sleep(2)

# bounds = [168, 135, 150, 145, 165, 180]

# for joint, bound in enumerate(bounds, start=0):
#     print(f'testing joint {joint}...')
#     for i in range(-bound, bound, 5):
#         angles = [0, 0, 0, 0, 0, 0]
#         angles[joint] = i
#         reached = sync_send_angles(angles, 50, timeout=4)
#         print(f'angle: {i}  {"FAILED" if not reached else ""}')