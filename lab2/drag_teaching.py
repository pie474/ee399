from pymycobot import MechArm270  

port = '/dev/ttyAMA0'

arm = MechArm270(port)


# unlock the arm to allow us to drag it around
arm.release_all_servos()


angles = arm.get_angles()

print(angles)

arm.set_gripper_calibration