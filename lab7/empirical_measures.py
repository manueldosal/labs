#!/usr/bin/env python3

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
from odometry import cozmo_drive_straight

def measure(robot: cozmo.robot.Robot):
    
    # Cozmo drove 440 mm and its front wheel did 5 loops. Therefore, 440 / 5 = 88 mm per loop.
	# Then radius = 88 / (2 * pi)
    print("Cozmo is driving straight for 440 mm")
    cozmo_drive_straight(robot, 440, 30)

    # Cozmo drove for 33 seconds with different speeds on each wheel.
    # Left wheel: 30 mm/s, Right wheel: 20 mm/s.
    # This generated a circle shape. The two circumferences formed with each wheel were computed.
    # C_L = 30 * 33 = 990 mm
    # C_R = 20 * 33 = 660 mm
    # Then each radius was computed.
    # r_L = 990 / (2 * pi) = 157.5
    # r_R = 660 / (2* pi) = 105
    # Then the radius is 157.5 - 105 = 52.5 mm
    print("Cozmo is driving around for 33 seconds")
    robot.drive_wheels(30, 20, duration=33)

if __name__ == '__main__':

	cozmo.run_program(measure)