#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
import math
from cozmo.util import degrees, Pose

def get_relative_pose(object_pose, reference_frame_pose):
	# ####
	# Implement computation of the relative frame using numpy.
	# Try to derive the equations yourself and verify by looking at
	# the books or slides before implementing.
	# ####

	#object_pose is the cube
	#reference_frame_pose is the robot
	xR = reference_frame_pose.position.x
	yR = reference_frame_pose.position.y
	zR = reference_frame_pose.rotation.angle_z.degrees
	xC = object_pose.position.x
	yC = object_pose.position.y
	zC = object_pose.rotation.angle_z.degrees

	#Debugging purposes----------------------------
	#print("xR:", xR, "yR:", yR, "zR:", zR, "xC:", xC, "yC:", yC, "zC:", zC)
	#---------------------------------------------

	positionX = (xC - xR) * math.cos(math.radians(zR)) + (yC - yR) * math.sin(math.radians(zR))
	positionY = - (xC - xR) * math.sin(math.radians(zR)) + (yC - yR) * math.cos(math.radians(zR))
	angleZ = zC - zR

	return Pose(positionX, positionY, 0, angle_z=degrees(angleZ))

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
