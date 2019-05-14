#! /usr/bin/env python
import rospy
import sys
import baxter_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from positionControl import *
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
import tf
import speech_recognition as sr

once = False

class gotomarker:
		def callback(self,data):
			global once
			if not data.markers:
				return
			self.place = 0
			self.get_pose(data)


		#detected markers are in an array
		#Here, we search for the marker with ID stored in variable 
		#self.place
		def get_pose(self, data):

			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.place:
					found_object = True
					self.marker_pose = data.markers[i].pose
					self.transform_marker_pose_to_robot_rf()
					self.move_to_marker()

		#Here, we transform the pose of the marker to the reference frame 'base'
		#which is the reference frame of the entire robot and from which
		#all other poses are relative to
		def transform_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.marker_pose.pose.position.y
			marker_pose.pose.position.x = self.marker_pose.pose.position.x
			marker_pose.pose.position.z = self.marker_pose.pose.position.z
			marker_pose.pose.orientation = self.marker_pose.pose.orientation

			tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))
			p = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)

			self.goal_pose.pose.position.y = p.pose.position.y
			self.goal_pose.pose.position.x = p.pose.position.x
			self.goal_pose.pose.position.z = p.pose.position.z
			self.goal_pose.pose.orientation.x = p.pose.orientation.x
			self.goal_pose.pose.orientation.y = p.pose.orientation.y
			self.goal_pose.pose.orientation.z = p.pose.orientation.z
			self.goal_pose.pose.orientation.w = p.pose.orientation.w

		#Calculates the distance between the current pose of the left gripper
		#and the goal_pose on all three axis and instructs the left gripper
		#to move on each of the axis by their respective distances til it gets to the
		#marker
		def move_to_marker(self):
			goal_pose = PoseStamped()
			goal_pose.pose.position.x = 0.3964
			goal_pose.pose.position.y = 1.06219
			goal_pose.pose.position.z = 0.35774

			goal_pose.pose.orientation.x = 0.966
			goal_pose.pose.orientation.y = -0.174
			goal_pose.pose.orientation.z = 0.10228
			goal_pose.pose.orientation.w = -0.1642
			# print goal_pose
			print "****************************"

		
			p = get_open_fridge_goal_pose(self.goal_pose)
			print p
			print "marker pose"
			print self.goal_pose
			# arm_pose = PoseStamped()
			# arm_pose.pose.position.y = p.pose.position.y
			# arm_pose.pose.position.x = p.pose.position.x
			# arm_pose.pose.position.z = p.pose.position.z
			# arm_pose.pose.orientation.x = p.pose.orientation.x
			# arm_pose.pose.orientation.y = p.pose.orientation.y
			# arm_pose.pose.orientation.z = p.pose.orientation.z
			# arm_pose.pose.orientation.w = p.pose.orientation.w

			move_to_goal_pose(self.lLimb, p, self.pause_event)
			playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			rospy.signal_shutdown("moving done")
			
			# if moveOnAxis(self.lLimb, 'y', y-0.05, y/4, self.pause_event):
			# 	print moveby
			# 	rospy.signal_shutdown("moving done")
			# current_pose = self.limb.endpoint_pose()
			# print "current pose"
			# print current_pose#['position']
			# print "marker pose"
			# print self.goal_pose.pose.position
					
			# 		#setting some offsets to the goal position for our convenience
			# 		goal_pose.pose.position.z -= 0.10
			# 		# goal_pose.pose.position.y -= 0.05

			# 		gripper_pose = self.limb.endpoint_pose()

			# 		goal_pose.pose.orientation.x = gripper_pose['orientation'][0]
			# 		goal_pose.pose.orientation.y = gripper_pose['orientation'][1]
			# 		goal_pose.pose.orientation.z = gripper_pose['orientation'][2]
			# 		goal_pose.pose.orientation.w = gripper_pose['orientation'][3]
					
			# 		#passing the pose goal into the moveit motion planner to plan the trajectory
			# 		left_arm.set_pose_target(goal_pose)
			# 		left_arm.set_start_state_to_current_state()
			# 		left_plan = left_arm.plan()
			# 		print "done planning"

			# 		#executing planned trajectory
			# 		rospy.sleep(5)
			# 		# left_arm.execute(left_plan)
    				
			# if not found_object:
			# 	print "Could not find marker ID "+ str(self.place)





		def move_to_origin(self):
			lLimb = baxter.Limb('left')
			rLimb = baxter.Limb('right')
			fPath = '/'
			pause_event = None
			playPositionFile(fPath, lLimb, rLimb, pause_event)
			rospy.sleep(3)



		def __init__(self):
			rospy.init_node('gotomarker', disable_signals=True)
			print "What can I do for you"
			self.limb = baxter.Limb('left')
			self.place = 0
			self.baxter_enabler = baxter.RobotEnable(versioned=True)
			self.baxter_enabler.enable()

			self.lLimb = baxter.Limb('left')
			self.rLimb = baxter.Limb('right')
			self.lGripper = baxter.Gripper('left')
			self.rGripper = baxter.Gripper('right')

			# calibrating gripper
			if not self.lGripper.calibrate():
			    print("left gripper did not calibrate")
			    sys.exit()

			self.lGripper.set_holding_force(100)
			self.lGripper.set_moving_force(100)

			self.rGripper.set_holding_force(100)
			self.rGripper.set_moving_force(100)

			self.goal_pose = PoseStamped()
			self.head = baxter.Head()
			self.marker_pose = None
			# self.head.set_pan(0)

			self.fridgeOpened = False
			self.pause_event = Event()
			sub = rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.callback)
			# if  self.marker_pose != None:
			# 	self.transform_marker_pose_to_robot_rf()
			# 	self.move_to_marker()
			# else:
			# 	print "Didn't get marker pose"
				#rospy.spin()
			rospy.spin()


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass


