from time import sleep
from pymycobot import MyCobotSocket
from kinematics import *

HOME = [0,0,0,0,0,0]
STRAIGHT_UP = [0,0,-90,0,0,0]

def connect_arm(ip='10.19.108.58'):
    arm = MyCobotSocket(ip, 9000)
    arm.connect_socket()
    sleep(2)
    return arm

def get_angles(arm):
    angles = None
    while not angles or angles == -1:
        try:
            angles = arm.get_angles()
        except:
            print('retrying angle getter')
    return angles