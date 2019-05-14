############# Baxter Speech Arm Positional Control ################

## Position Control Package

## Written by the Robotic Assistance research team
## Tufts University

## 2018

###################################################################
# 
# Provides a set of functions that are useful for controlling the 
# arm of the baxter robot in 3d space.

# See function comments for details.


import argparse
import sys
import struct
import time
import json
import matplotlib.pyplot as plt
from threading import *

import rospy

from math import *

from std_msgs.msg import (
    UInt16,
)

from StringIO import StringIO

import baxter_interface as baxter

import speech_recognition as SR

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from scipy.interpolate import CubicSpline

def xyzToAngles(limbs, x, y, z, xr, yr, zr, wr):
	# converts cartesian and quaternion orientation in space to a 
	# dictionary of joint angles using the baxter sdk inverse kinematics service

	# limbs: string name of lime ('left' or 'right')
	# x, y, z: endpoint cartesian position
	# xr, yr, zr, wr: quaternion elements describing orientation of endpoint in space


	ns = "ExternalTools/" + limbs + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	pose = PoseStamped(
	            header=hdr,
	            pose=Pose(
	                position=Point(
	                    x=x,
	                    y=y,
	                    z=z,
	                ),
	                orientation=Quaternion(
	                    x=xr,
	                    y=yr,
	                    z=zr,
	                    w=wr,
	                ),
	            ),
	        )

	ikreq.pose_stamp.append(pose)
	try:
	    rospy.wait_for_service(ns, 5.0)
	    resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
	    rospy.logerr("Service call failed: %s" % (e,))
	    exit()

	resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
	                               resp.result_type)
	if (resp_seeds[0] != resp.RESULT_INVALID):
	    seed_str = {
	                ikreq.SEED_USER: 'User Provided Seed',
	                ikreq.SEED_CURRENT: 'Current Joint Angles',
	                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
	               }.get(resp_seeds[0], 'None')
	    # Format solution into Limb API-compatible dictionary
	    limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
	    return limb_joints
	else:
	    print("INVALID POSE - No Valid Joint Solution Found.")
	    return "invalid"


def euler2Quat(xr, yr, zr):
	# converts euler angles to a quaternion using mathematical conversion
	# xr, yr, zr: euler angles of rotation around the end effector's base axes 

	toRet = {'qw': 0, 'qx': 0, 'qy': 0, 'qz': 0}

	xr = radians(xr)
	yr = radians(yr)
	zr = radians(zr)

	c1 = cos(yr/2)
	c2 = cos(zr/2)
	c3 = cos(xr/2)
	s1 = sin(yr/2)
	s2 = sin(zr/2)
	s3 = sin(xr/2)
	

	toRet['qw'] = c1*c2*c3 - s1*s2*s3
	toRet['qx'] = s1*s2*c3 + c1*c2*s3
	toRet['qy'] = s1*c2*c3 + c1*s2*s3
	toRet['qz'] = c1*s2*c3 - s1*c2*s3

	return toRet

def moveOnAxis(limb, axis, dist, speed, pause_event):
## Moves arm on x, y, or z axis keeping orientation constant
# speed is in m/s
# dist in m
# limb is a handle to a limb object


	if 'left' in limb.joint_names()[0]: limbName = 'left'
	else: limbName = 'right'
	
	position = {'x':0, 'y':1, 'z':2}
	pose = limb.endpoint_pose()

	position['x'] = pose['position'][0]
	position['y'] = pose['position'][1]
	position['z'] = pose['position'][2]
	orient = pose['orientation']

	secPframe = .05
	frames = int(abs(dist)*(1/float(speed))*(1/secPframe))
	if frames == 0: return limb.endpoint_pose()
	distPframe = float(dist)/float(frames)

	limb.set_joint_position_speed(1)

	rate = rospy.Rate(1/secPframe)
	for i in range(0, frames):
		waitForNotPause(pause_event)
		position[axis] += distPframe
		jointPos = xyzToAngles(limbName, position['x'], position['y'], position['z'], orient[0], orient[1], orient[2], orient[3])
		if jointPos != "invalid":
			# Check if it is minor move. if it is not, use smoother movement function
			minorMove = True
			actualJointPos = limb.joint_angles()
			for joint, angle in jointPos.iteritems():
				if abs(angle-actualJointPos[joint]) > .8: minorMove = False
			if minorMove:
				limb.set_joint_positions(jointPos)
			else:
				return False#limb.endpoint_pose()
				# limb.move_to_joint_positions(jointPos, timeout=3, threshold=.02)
		else:
			print("Can't Move Here")
			return False#limb.endpoint_pose()
		rate.sleep()

	return True#limb.endpoint_pose()

def playPositionFile(fPath, lLimb, rLimb, pause_event):
	# Moves limb to specified joint positions
	# fPath: string indentifying path to file
	# lLimb: handle to the left limb 'Limb' object
	# rLimb: handle to the right limb 'Limb' object
	
	with open(fPath, 'r') as f:
		fText = f.read()
		
	wpArray = json.loads(fText)
	initPoint = json.loads(json.dumps(lLimb.joint_angles()))
	wpArray.insert(0, initPoint)
	path, period = splineWaypoints(wpArray)

	lLimb.set_joint_position_speed(.7)
	rLimb.set_joint_position_speed(.7)

	rate = rospy.Rate(1/period)
	for i in range(len(path)):
		waitForNotPause(pause_event)
		if path[i] != "invalid":
			lLimb.set_joint_positions(path[i])
		rate.sleep()


def playRightPositionFile(fPath, lLimb, rLimb, pause_event):
	# Moves limb to specified joint positions
	# fPath: string indentifying path to file
	# lLimb: handle to the left limb 'Limb' object
	# rLimb: handle to the right limb 'Limb' object
	
	with open(fPath, 'r') as f:
		fText = f.read()
		
	wpArray = json.loads(fText)
	initPoint = json.loads(json.dumps(rLimb.joint_angles()))
	wpArray.insert(0, initPoint)
	path, period = splineWaypoints(wpArray)

	lLimb.set_joint_position_speed(.7)
	rLimb.set_joint_position_speed(.7)

	rate = rospy.Rate(1/period)
	for i in range(len(path)):
		waitForNotPause(pause_event)
		if path[i] != "invalid":
			rLimb.set_joint_positions(path[i])
		rate.sleep()

def setOrientation(limb, xr, yr, zr, wr):
	
	if 'left' in limb.joint_names()[0]: limbName = 'left'
	else: limbName = 'right'

	pose = limb.endpoint_pose()

	x = pose['position'][0]
	y = pose['position'][1]
	z = pose['position'][2]
	
	jointPos = xyzToAngles(limbName, x, y, z, xr, yr, zr, wr)
	limb.move_to_joint_positions(jointPos, timeout=3, threshold=.002)


#########################################################################
############################   Utilities   ##############################
#########################################################################

def splineWaypoints(wpArray):
	# creates clamped cubic spline of waypoints in wpArray

	nPts = len(wpArray)
	splinePtsPerwp = 50
	splinePts = nPts*splinePtsPerwp
	deltPts = 1
	splineTime = float(deltPts*(nPts - 1))
	period = splineTime/splinePts
	
	splinedStatesvTime = {}
	x = frange(0, splineTime, deltPts)
	xspline = frange(0, splineTime, splineTime/splinePts)
	for key in wpArray[0]:
		spline = CubicSpline(x, [Point[key] for Point in wpArray], bc_type=((1, 0.0),(1, 0.0)))
		splinedState = spline(xspline)
		splinedStatesvTime[key] = splinedState

	jointPath = []
	jointPos = {}
	for i in range(len(xspline)):
		for key in wpArray[0]:
			jointPos[key] = splinedStatesvTime[key][i]
		jointPath.append(jointPos)
		jointPos = {}

	# plt.figure(figsize=(6.5, 4))
	# for key in wpArray[0]:
	# 	plt.plot(x, [Point[key] for Point in wpArray], 'o', label='{}-waypoint'.format(key))
	# 	plt.plot(xspline, [Point[key] for Point in jointPath], label='{}-trajectory'.format(key))
	# plt.xlabel('Time (s)')
	# plt.ylabel('Joint Angle (rad)')
	# plt.title('Joint Angle Waypoints and Cubic Splines vs. Time')
	# plt.legend(loc='best')
	# plt.axis([-.5, 17, -3.5, 2.5])
	# plt.show()

	return (jointPath, period)



#This function is an adaptation of the splineWaypoints function above
#It produces a much smoother spline for smaller waypoint arrays
#To avoid jerking during arm motion
def splineForArmMotionWithImageProc(wpArray):
	nPts = len(wpArray)
	splinePtsPerwp = 50
	splinePts = nPts*splinePtsPerwp
	deltPts = 1
	splineTime = float(deltPts*(nPts - 1))
	period = splineTime/splinePts
	
	splinedStatesvTime = {}
	x = frange(0, splineTime, deltPts)
	xspline = frange(0, splineTime, splineTime/splinePts)
	for key in wpArray[0]:
		spline = CubicSpline(x, [Point[key] for Point in wpArray], bc_type=((1, 0.0),(1, 0.0)))
		splinedState = spline(xspline)
		splinedStatesvTime[key] = splinedState

	jointPath = []
	jointPos = {}
	for i in range(len(xspline)):
		for key in wpArray[0]:
			jointPos[key] = splinedStatesvTime[key][i]
		jointPath.append(jointPos)
		jointPos = {}
	return (jointPath, period)


#This function takes in the pose of the fridge marker
#and computes the pose the arm should have in order to 
#open the fridge
#The literal values were measured with regards to a specific
#fridge and a specific position of the marker on the
#fridge
def get_open_fridge_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.18948
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.06993
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.1471

	goal_pose.pose.orientation.x = 0.98933
	goal_pose.pose.orientation.y = -0.05598
	goal_pose.pose.orientation.z = 0.04486
	goal_pose.pose.orientation.w = -0.12683

	return goal_pose

def get_open_microwave_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.02182
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.08337
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.15917

	goal_pose.pose.orientation.x = 0.9714
	goal_pose.pose.orientation.y = -0.00848
	goal_pose.pose.orientation.z = 0.02324
	goal_pose.pose.orientation.w = -0.2363

	return goal_pose


def get_new_open_microwave_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x + 0.063
	goal_pose.pose.position.y = marker_pose.pose.position.y - 0.0142
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.1455

	goal_pose.pose.orientation.x = -0.6499
	goal_pose.pose.orientation.y = 0.7276
	goal_pose.pose.orientation.z = -0.1987
	goal_pose.pose.orientation.w = 0.0939

	return goal_pose

def get_pick_bottle_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x + 0.0121
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.04184
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.2314

	goal_pose.pose.orientation.x = -0.46362
	goal_pose.pose.orientation.y = -0.4839
	goal_pose.pose.orientation.z = -0.54194
	goal_pose.pose.orientation.w = 0.5071

	return goal_pose

def get_fridge_grab_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.01222
	goal_pose.pose.position.y = marker_pose.pose.position.y - 0.3961
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.1908#0.28453

	goal_pose.pose.orientation.x = 0.4097
	goal_pose.pose.orientation.y = 0.5572
	goal_pose.pose.orientation.z = 0.53629
	goal_pose.pose.orientation.w = -0.483696

	return goal_pose


def get_pick_bowl_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.0351
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.0348
	goal_pose.pose.position.z = marker_pose.pose.position.z + 0.0118

	goal_pose.pose.orientation.x = -0.6738
	goal_pose.pose.orientation.y = 0.033
	goal_pose.pose.orientation.z = -0.05377
	goal_pose.pose.orientation.w = 0.736

	return goal_pose

def get_pick_from_floor_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.701
	goal_pose.pose.position.y = marker_pose.pose.position.y - 0.088
	goal_pose.pose.position.z = -0.463#marker_pose.pose.position.z - 0.772

	return goal_pose


def get_get_food_in_microwave_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.3265
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.128
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.11#307

	goal_pose.pose.orientation.x = 0.026
	goal_pose.pose.orientation.y = 0.6931
	goal_pose.pose.orientation.z = 0.72
	goal_pose.pose.orientation.w = -0.0226

	return goal_pose

def get_put_food_in_microwave_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.34733
	goal_pose.pose.position.y = marker_pose.pose.position.y - 0.12933
	goal_pose.pose.position.z = 0.4419#marker_pose.pose.position.z - 0.17897

	goal_pose.pose.orientation.x = 0.0924
	goal_pose.pose.orientation.y = 0.6189
	goal_pose.pose.orientation.z = 0.7768
	goal_pose.pose.orientation.w = -0.0708

	return goal_pose

def get_take_from_shelf_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x + 0.0986
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.10
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.0997

	goal_pose.pose.orientation.x = -0.1664
	goal_pose.pose.orientation.y = 0.6512
	goal_pose.pose.orientation.z = 0.0515
	goal_pose.pose.orientation.w = 0.739

	return goal_pose


def get_pick_bottle_drink_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.01451
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.02333
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.19003

	goal_pose.pose.orientation.x = 0.67597
	goal_pose.pose.orientation.y = -0.05845
	goal_pose.pose.orientation.z = 0.7311432
	goal_pose.pose.orientation.w = 0.07310
	return goal_pose


def get_pour_drink_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.032
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.1635
	goal_pose.pose.position.z = marker_pose.pose.position.z + 0.0527

	goal_pose.pose.orientation.x = -0.3384
	goal_pose.pose.orientation.y = 0.5951
	goal_pose.pose.orientation.z = -0.5406
	goal_pose.pose.orientation.w = 0.4889
	return goal_pose



def get_move_cup_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x + 0.0251
	goal_pose.pose.position.y = marker_pose.pose.position.y - 0.012
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.134

	goal_pose.pose.orientation.x = -0.0219
	goal_pose.pose.orientation.y = 0.7828
	goal_pose.pose.orientation.z = 0.0167
	goal_pose.pose.orientation.w = 0.6216
	return goal_pose



def get_cook_for_seconds_goal_pose(marker_pose):
	goal_pose = PoseStamped()
	goal_pose.pose.position.x = marker_pose.pose.position.x - 0.0182
	goal_pose.pose.position.y = marker_pose.pose.position.y + 0.02982
	goal_pose.pose.position.z = marker_pose.pose.position.z - 0.26137

	goal_pose.pose.orientation.x = -0.6789
	goal_pose.pose.orientation.y = -0.02667
	goal_pose.pose.orientation.z = 0.00525
	goal_pose.pose.orientation.w = 0.73373

	return goal_pose


#Takes in goal pose and moves arm to the goal
def move_to_goal_pose(limb, goal_pose, pause_event):
	wp_array = []
	goal_pose_wp = xyzToAngles('left', goal_pose.pose.position.x, goal_pose.pose.position.y,
	goal_pose.pose.position.z, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
	goal_pose.pose.orientation.z, goal_pose.pose.orientation.w)
	init_wp = limb.joint_angles()
	wp_array.append(init_wp)
	if goal_pose_wp != "invalid":
		wp_array.append(goal_pose_wp)
	else:
		wp_array.append(init_wp)

	path, period = splineForArmMotionWithImageProc(wp_array)

	limb.set_joint_position_speed(.7)

	rate = rospy.Rate(50)
	for i in range(len(path)):
		waitForNotPause(pause_event)
		if path[i] != "invalid":
			limb.set_joint_positions(path[i])
		rate.sleep()

def right_move_to_goal_pose(limb, goal_pose, pause_event):
	wp_array = []
	goal_pose_wp = xyzToAngles('right', goal_pose.pose.position.x, goal_pose.pose.position.y,
	goal_pose.pose.position.z, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
	goal_pose.pose.orientation.z, goal_pose.pose.orientation.w)
	init_wp = limb.joint_angles()
	wp_array.append(init_wp)
	if goal_pose_wp != "invalid":
		wp_array.append(goal_pose_wp)
	else:
		wp_array.append(init_wp)

	path, period = splineForArmMotionWithImageProc(wp_array)

	limb.set_joint_position_speed(.7)

	rate = rospy.Rate(50)
	for i in range(len(path)):
		waitForNotPause(pause_event)
		if path[i] != "invalid":
			limb.set_joint_positions(path[i])
		rate.sleep()


#Takes in goal joint angle dict and moves arm to the goal
def move_to_goal_joint_angle(limb, joint_angles, pause_event):
	wp_array = []
	init_wp = limb.joint_angles()
	wp_array.append(init_wp)
	wp_array.append(joint_angles)

	path, period = splineForArmMotionWithImageProc(wp_array)

	limb.set_joint_position_speed(.7)

	rate = rospy.Rate(50)
	for i in range(len(path)):
		waitForNotPause(pause_event)
		if path[i] != "invalid":
			limb.set_joint_positions(path[i])
		rate.sleep()




def frange(start, stop, step):
	i = start
	toRet = []
	while i <= stop:
		toRet.append(i)
		i += step

	return toRet

def waitForNotPause(pause_event):
	if pause_event.isSet():
		while pause_event.isSet():
			pass
