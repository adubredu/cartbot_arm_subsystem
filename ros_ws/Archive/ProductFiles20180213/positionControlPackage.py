import argparse
import sys
import struct
import time
import json

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

def xyzToAngles(limbs, x, y, z, xr, yr, zr, wr):

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

def moveOnAxis(limb, axis, dist, speed):
## Moves arm on x, y, or z axis keeping orientation constant
# speed is in m/s
# dist in m
# limb is a handle to a limb object


	if 'left' in limb.joint_names()[0]: limbName = 'left'
	else: limbName = 'right'
	
	print(limbName)

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
				print('bigmove')
				limb.move_to_joint_positions(jointPos, timeout=3, threshold=.02)
		else:
			print("Can't Move Here")
			return limb.endpoint_pose()
		rate.sleep()

	return limb.endpoint_pose()

def playPositionFile(fPath, lLimb, rLimb):
	# Moves limb to specified joint positions
	# fPath: string indentifying path to file
	# lLimb handle to the left limb 'Limb' object
	# rLimb hanld to the right limb 'Limb' object
	
	with open(fPath, 'r') as f:
		fText = f.read()
		
	fText = fText.replace("'", '"')
	wpArray = json.loads(fText)

	lLimb.set_joint_position_speed(.5)
	rLimb.set_joint_position_speed(.5)

	rate = rospy.Rate(1000)
	for wp in wpArray:
		lPos = wp['left']
	 	rPos = wp['right']
	 	# move left 
	 	if lPos != '':
	 		lLimb.move_to_joint_positions(lPos)
	 	if rPos != '':
	 		rLimb.move_to_joint_positions(rPos)

	return (lLimb.endpoint_pose(), rLimb.endpoint_pose)