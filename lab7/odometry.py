#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import asyncio

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# ####
	# Empirically determine the radius of the robot's front wheel using the
	# cozmo_drive_straight() function. You can write a separate script for doing 
	# experiments to determine the radius. This function should return the radius
	# in millimeters. Write a comment that explains how you determined it and any
	# computation you do as part of this function.
	# ####

	# The radius was determined using empirical_measures.py.
	# Cozmo drove 440 mm and its front wheel did 5 loops. Therefore, 440 / 5 = 88 mm per loop.
	# Then radius = 88 / (2 * pi)
	return 14

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# Empirically determine the distance between the wheels of the robot using
	# robot.drive_wheels() function. Write a comment that explains how you determined
	# it and any computation you do as part of this function.
	# ####

	# This was computed using empirical_measures.py
	# Cozmo drove for 33 seconds with different speeds on each wheel.
    # Left wheel: 30 mm/s, Right wheel: 20 mm/s.
    # This generated a circle shape. The two circumferences formed with each wheel were computed.
    # C_L = 30 * 33 = 990 mm
    # C_R = 20 * 33 = 660 mm
    # Then each radius was computed.
    # r_L = 990 / (2 * pi) = 157.5
    # r_R = 660 / (2* pi) = 105
    # Then the radius is 157.5 - 105 = 52.5 mm
	return 52.5

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	# ####
	# Implement this function.
	# ####

	# Let's make speed to be 30 mm/s
	speed = 30

	# Compute the distance based on the angle
	dist = 2 * get_front_wheel_radius() * math.pi * angle_deg / 360
	cozmo_drive_straight(robot, dist, speed)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	# ####
	# Implement your version of a driving straight function using the
	# robot.drive_wheels() function.
	# ####

	print("My Drive straight:", "dist", dist, "speed", speed)
	
	# duration = dist / speed
	robot.drive_wheels(speed, speed, duration= dist / speed + 0.65)

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	# ####
	# Implement your version of a rotating in place function using the
	# robot.drive_wheels() function.
	# ####

	# Let's compute the speed in terms of the wheels in mm / s
	wheelSpeedFactor = math.pi * get_distance_between_wheels() / 360 + 0.31
	rightwheelSpeed = wheelSpeedFactor * speed if angle > 0 else - wheelSpeedFactor * speed
	time = abs(angle) / speed + 0.65

	print("My Turn In Place:", "angle", angle, "speed", speed, "rightwheelSpeed", rightwheelSpeed, "time", time)

	robot.drive_wheels(-rightwheelSpeed, rightwheelSpeed, duration=time)

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions. This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####

	print("My Go To Pose 1:", "x", x, "y", y, "angle_z", angle_z)

	speed = 30
	angularSpeed = 30

	# First turn to face the target
	angleToFaceTarget = math.atan2(y, x) * 180 / math.pi
	my_turn_in_place(robot, angleToFaceTarget, angularSpeed)

	time.sleep(.1)

	# Second go straight until the target point
	distanceToTravel = math.sqrt(x * x + y * y)
	my_drive_straight(robot, distanceToTravel, speed)

	time.sleep(.1)

	# Third turn to get correct rotation
	angleToFinish = angle_z - angleToFaceTarget
	my_turn_in_place(robot, angleToFinish, angularSpeed)


def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the 
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####

	timePerIteration = 3
	p1 = 0.5
	p2 = 0.5
	p3 = 0.5
	errorDistance = 5
	errorAngleInRadians = math.radians(10)
	radius = get_front_wheel_radius()
	distanceWheels = get_distance_between_wheels()

	#Initial values for robot
	xR = 0
	yR = 0
	thetaInRadians = 0
	

	while True:
		distance = math.sqrt(math.pow(xR - x, 2) + math.pow(yR - y, 2))
		bearing = math.atan2(y - yR, x - xR) - thetaInRadians
		heading = math.radians(angle_z) - thetaInRadians

		#Stop if we are in the desired pose
		if distance < errorDistance and abs(heading) < errorAngleInRadians:
			break

		diffX = p1 * distance
		diffTheta = p2*bearing + p3*heading

		#Get wheels rotation
		rotationLeft = (2 * diffX - diffTheta * distanceWheels) / (2 * radius)
		rotationRight = (2 * diffX + diffTheta * distanceWheels) / (2 * radius)
		print("Rotation. RotationLeft:", rotationLeft, "RotationRight:", rotationRight)
		
		#Compute the distance from the wheels rotation
		distanceLeft = rotationLeft * radius
		distanceRight = rotationRight * radius
		print("Distance to drive. Left:", distanceLeft, ". Right:", distanceRight)

		#Drive robot using timePerIteration
		speedLeft = distanceLeft / timePerIteration
		speedRight = distanceRight / timePerIteration
		print("Speed. Left", speedLeft, "Right", speedRight)
		robot.drive_wheels(speedLeft, speedRight, duration=timePerIteration + 0.65)
		time.sleep(.1)

		#Update robot pose
		thetaInRadians += radius / distanceWheels * (rotationRight - rotationLeft)
		xR += radius / 2 * (rotationLeft + rotationRight) * math.cos(bearing)
		yR += radius / 2 * (rotationLeft + rotationRight) * math.sin(bearing)
		print("New position. X:", xR, "Y:", yR, "Theta:", math.degrees(thetaInRadians))
		print()

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# Implement a function that makes the robot move to a desired pose
	# as fast as possible. You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	if(x < 0):
		#Do approach 1
		my_go_to_pose1(robot, x, y, angle_z)
	else:
		#Do approach 2
		my_go_to_pose2(robot, x, y, angle_z)

def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	cozmo_drive_straight(robot, 62, 50)
	cozmo_turn_in_place(robot, 60, 30)
	cozmo_go_to_pose(robot, 100, 100, 45)

	rotate_front_wheel(robot, 90)
	my_drive_straight(robot, 62, 50)
	my_turn_in_place(robot, 90, 30)

	my_go_to_pose1(robot, 100, 100, 45)
	my_go_to_pose2(robot, 100, 100, 45)
	my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	cozmo.run_program(run)



