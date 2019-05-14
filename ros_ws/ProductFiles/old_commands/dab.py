#! /usr/bin/env python

import copy
import rospy
import sys
from positionControl import *
import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

import IPython
import tf
import time

def main():
	#Initialize moveit_commander
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_node')
	# limb = baxter.Limb('left')
	#Start a node
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	left_arm = moveit_commander.MoveGroupCommander('left_arm')
	left_arm.set_planner_id('RRTConnectkConfigDefault')
	left_arm.set_planning_time(10)


	#Start tf node
	# listener = tf.TransformListener()
	# from_frame = 'head_camera'
	# to_frame = 'camera_link'
	# time.sleep(1)
	# if not listener.canTransform(to_frame,from_frame,(rospy.Time.now()-rospy.Duration(1000))):
	#     print 'Frames not found or cannot be transformed'
	#     print 'Did you place AR marker 0 within view of the baxter left hand camera?'
	#     exit(0)


	# left_arm = baxter_interface.Limb('left')
	
	# #Initialize the left limb for joint velocity control
	# print 'here'
	# current_pose = limb.endpoint_pose()
	# print current_pose
	# print marker_pose
	# listener.waitForTransform(from_frame, to_frame, rospy.Time.now(), rospy.Duration(5.0))
	# now = rospy.Time.now() - rospy.Duration(5.0)
	# t = listener.getLatestCommonTime(from_frame, to_frame)
	# listener.waitForTransform(from_frame, to_frame, now, rospy.Duration(5.0))
	# position, quaternion = listener.lookupTransform(from_frame, to_frame, now)

	
	limb = baxter_interface.Limb('left')
	position = limb.endpoint_pose()
	print position
	goal_pose = PoseStamped()
	goal_pose.header.frame_id = "base"
	goal_pose.pose.position.x = position['position'][0]
	goal_pose.pose.position.y = position['position'][1]
	goal_pose.pose.position.z = position['position'][2]+0.1

	goal_pose.pose.orientation.x = position['orientation'][0]
	goal_pose.pose.orientation.y = position['orientation'][1]
	goal_pose.pose.orientation.z = position['orientation'][2]
	goal_pose.pose.orientation.w = position['orientation'][3]
	print goal_pose

	left_arm.set_pose_target(goal_pose)
	left_arm.set_start_state_to_current_state()

	left_plan = left_arm.plan()
	rospy.sleep(5)
	# raw_input('Press <Enter> to move: ')
	left_arm.execute(left_plan)



	# goal_joint_angles = xyzToAngles('left', goal_pose.pose.position.x, 
	# 	goal_pose.pose.position.y, goal_pose.pose.position.z,
	# 	current_pose['orientation'][0], current_pose['orientation'][1], 
	# 	current_pose['orientation'][2], current_pose['orientation'][3])

	# if goal_joint_angles != 'invalid':
	# 	current_joint_angles = limb.joint_angles()
	# 	wpArray = []
	# 	wpArray.append(json.loads(json.dumps(current_joint_angles)))
	# 	wpArray.append(json.loads(json.dumps(goal_joint_angles)))

	# 	path, period = splineWaypoints(wpArray)

	# 	limb.set_joint_position_speed(.7)

	# 	rate = rospy.Rate(1/(period*2))

	# 	for i in range(len(path)):
	# 		if path[i] != 'invalid':
	# 			limb.set_joint_positions(path[i])
	# 			rate.sleep()
if __name__ == '__main__':
    main()