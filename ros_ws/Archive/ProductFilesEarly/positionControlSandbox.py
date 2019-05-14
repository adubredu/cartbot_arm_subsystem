import argparse
import sys
import struct

import rospy

from std_msgs.msg import (
    UInt16,
)

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

def xyzToAngles(x, y, z):

	ns = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	poseLeft = PoseStamped(
	            header=hdr,
	            pose=Pose(
	                position=Point(
	                    x=x,
	                    y=y,
	                    z=z,
	                ),
	                orientation=Quaternion(
	                    x=-0.5,
	                    y=-0.5,
	                    z=-0.5,
	                    w=0.5,
	                ),
	            ),
	        )

	ikreq.pose_stamp.append(poseLeft)
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