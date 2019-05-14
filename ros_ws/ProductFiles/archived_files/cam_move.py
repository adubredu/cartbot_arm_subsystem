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

		#######   FRIDGE OPERATIONS   ##############
		def open_fridge_callback(self,data):
			if not self.fridgeOpened:
				global once
				if not data.markers:
					return
				self.get_fridge_pose(data)


		#detected markers are in an array
		#Here, we search for the marker with ID stored in variable 
		#self.place
		def get_fridge_pose(self, data):
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.fridge_marker:
					found_object = True
					self.fridge_marker_pose = data.markers[i].pose
					self.transform_fridge_marker_pose_to_robot_rf()
					# print self.fridge_goal_pose
					self.open_fridge()




		#Here, we transform the pose of the marker to the reference frame 'base'
		#which is the reference frame of the entire robot and from which
		#all other poses are relative to
		def transform_fridge_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.fridge_marker_pose.pose.position.y
			marker_pose.pose.position.x = self.fridge_marker_pose.pose.position.x
			marker_pose.pose.position.z = self.fridge_marker_pose.pose.position.z
			marker_pose.pose.orientation = self.fridge_marker_pose.pose.orientation

			tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))
			self.fridge_goal_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)

			# self.goal_pose.pose.position.y = p.pose.position.y
			# self.goal_pose.pose.position.x = p.pose.position.x
			# self.goal_pose.pose.position.z = p.pose.position.z
			# self.goal_pose.pose.orientation.x = p.pose.orientation.x
			# self.goal_pose.pose.orientation.y = p.pose.orientation.y
			# self.goal_pose.pose.orientation.z = p.pose.orientation.z
			# self.goal_pose.pose.orientation.w = p.pose.orientation.w


		#Calculates the distance between the current pose of the left gripper
		#and the goal_pose on all three axis and instructs the left gripper
		#to move on each of the axis by their respective distances til it gets to the
		#marker
		def open_fridge(self):
			p = get_open_fridge_goal_pose(self.fridge_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			#opening motion
			ang =  self.lLimb.joint_angles()
			ang['left_w1']+=1.3
			move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)

			# g = self.get_fridge_grab_goal_pose(self.bowl_marker)
			# move_to_goal_pose(self.lLimb, g, self.pause_event)
			# playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# self.fridgeOpened = True
			rospy.signal_shutdown("moving done")




		##############   MICROWAVE OPERATIONS   #################

		def transform_microwave_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.microwave_marker_pose.pose.position.y
			marker_pose.pose.position.x = self.microwave_marker_pose.pose.position.x
			marker_pose.pose.position.z = self.microwave_marker_pose.pose.position.z
			marker_pose.pose.orientation = self.microwave_marker_pose.pose.orientation

			tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))
			self.microwave_goal_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)


		def open_microwave_callback(self, data):
			if not data.markers:
				return
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.microwave_marker:
					self.microwave_marker_pose = data.markers[i].pose
					self.transform_microwave_marker_pose_to_robot_rf()
					print self.microwave_goal_pose
					self.open_microwave()


		def open_microwave(self):
			p = get_open_microwave_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			ang =  self.lLimb.joint_angles()
			ang['left_w0']+=0.5
			move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)

			#move to origin 
			# move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			# ang =  self.lLimb.joint_angles()
			# ang['left_w1']+=0.8
			# move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			# # playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# # self.fridgeOpened = True
			rospy.signal_shutdown("moving done")




		#################   BOTTLE OPERATIONS   ######################

		def transform_bottle_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.bottle_pose.pose.position.y
			marker_pose.pose.position.x = self.bottle_pose.pose.position.x
			marker_pose.pose.position.z = self.bottle_pose.pose.position.z
			marker_pose.pose.orientation = self.bottle_pose.pose.orientation

			tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))
			self.bottle_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)

			


		def pick_bottle_callback(self, data):
			if not data.markers:
				return
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.bottle_marker:
					self.bottle_pose = data.markers[i].pose
					self.transform_bottle_marker_pose_to_robot_rf()
					# print self.bottle_pose
					# self.grab_bottle()
					self.pick_bottle()

		
		def pick_bottle(self):
			p = get_pick_bottle_goal_pose(self.bottle_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			#opening motion
			# ang =  self.lLimb.joint_angles()
			# ang['left_w1']+=1.3
			# move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			# playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# self.fridgeOpened = True
			rospy.signal_shutdown("moving done")

		#deprecated
		def grab_bottle(self):
			e1 = 1.15
			s0 = -0.5
			s1 = 0.15

			x = self.bottle_pose.pose.position.x
			y = self.bottle_pose.pose.position.y
			z = self.bottle_pose.pose.position.z

			if y >0.25:
				s1+= 0.05
			elif y < 0.16:
				s1 -= 0.05

			if z>0.25:
				s0+=0.05
			elif z < 0.19:
				s0-= 0.05

			e_out = self.valmap(x, -0.008, 0.111, 0.08, -0.1)
			e1+=e_out 


			goal = {'left_w0': -0.34399519168330406, 
			'left_w1': 0.27880100819817394, 
			'left_w2': -1.5, 
			'left_e0': 1.7725148004015956, 
			'left_e1': e1, 
			'left_s0': s0, 
			'left_s1': s1
			}
		
			print goal
			move_to_goal_joint_angle(self.lLimb, goal, self.pause_event)
			




		#################   BOWL OPERATIONS   ######################

		def transform_bowl_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.bowl_pose.pose.position.y
			marker_pose.pose.position.x = self.bowl_pose.pose.position.x
			marker_pose.pose.position.z = self.bowl_pose.pose.position.z
			marker_pose.pose.orientation = self.bowl_pose.pose.orientation

			tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))
			self.bowl_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)

			


		def pick_bowl_callback(self, data):
			if not data.markers:
				return
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.bowl_marker:
					self.bowl_pose = data.markers[i].pose
					self.transform_bowl_marker_pose_to_robot_rf()
					print self.bowl_pose
					# self.grab_bottle()
					self.pick_bowl()

		
		def pick_bowl(self):
			# p = get_pick_bowl_goal_pose(self.bowl_pose)
			p = get_fridge_grab_goal_pose(self.bowl_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			#opening motion
			# ang =  self.lLimb.joint_angles()
			# ang['left_w1']+=1.3
			# move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			# playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# self.fridgeOpened = True
			rospy.signal_shutdown("moving done")




		def move_to_origin(self):
			lLimb = baxter.Limb('left')
			rLimb = baxter.Limb('right')
			fPath = '/'
			pause_event = None
			playPositionFile(fPath, lLimb, rLimb, pause_event)
			rospy.sleep(3)


		def valmap(self,value, istart, istop, ostart, ostop):
  			return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

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
			# if not self.lGripper.calibrate():
			#     print("left gripper did not calibrate")
			#     sys.exit()

			self.lGripper.set_holding_force(100)
			self.lGripper.set_moving_force(100)

			self.rGripper.set_holding_force(100)
			self.rGripper.set_moving_force(100)
			# self.lGripper.open()
			self.fridge_goal_pose = PoseStamped()
			self.head = baxter.Head()
			self.marker_pose = None
			# self.head.set_pan(0)

			self.fridgeOpened = False
			self.fridge_marker = 0

			self.bottleGrabbed = False
			self.bottle_marker = 4

			self.microwaveOpened = False
			self.microwave_marker = 1

			self.bottle_pose = None
			self.pause_event = Event()

			self.bowlGrabbed = False
			self.bowl_pose = None
			self.bowl_marker = 3

			self.origin = {'left_w0': -0.3156165471074239, 
			'left_w1': 1.1662088939898858, 
			'left_w2': -1.0814564554592168, 
			'left_e0': 0.3041116911982833, 
			'left_e1': 0.635835036578504, 
			'left_s0': 0.7359272829880272, 
			'left_s1': -0.8141603031701834
			}

			self.fridge_grab_pose = PoseStamped()
			self.fridge_grab_pose.pose.position.x = 0.58813
			self.fridge_grab_pose.pose.position.y = 0.8716
			self.fridge_grab_pose.pose.position.z = 0.60607

			self.fridge_grab_pose.pose.orientation.x = 0.4097
			self.fridge_grab_pose.pose.orientation.y = 0.5572
			self.fridge_grab_pose.pose.orientation.z = 0.53629
			self.fridge_grab_pose.pose.orientation.w = -0.483696

			# print self.lLimb.endpoint_pose()
			# print self.lLimb.joint_angles()
			# move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			# rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.open_fridge_callback)
			rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.open_microwave_callback)
			# rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.pick_bottle_callback)
			# rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.pick_bowl_callback)
			# move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			# rospy.Subscriber('/left_hand_camera/ar_pose_marker', AlvarMarkers, self.pick_bottle_callback)
			rospy.spin()
			# if  self.marker_pose != None:
			# 	self.transform_marker_pose_to_robot_rf()
			# 	self.move_to_marker()
			# else:
			# 	print "Didn't get marker pose"
				#rospy.spin()

			# p = PoseStamped()
			# p.pose.position.y = 0.4976
			# p.pose.position.x = 1.215
			# p.pose.position.z = 0.45597

			# p.pose.orientation.x = -0.549
			# p.pose.orientation.y = 0.469
			# p.pose.orientation.z = 0.4772
			# p.pose.orientation.w = 0.5
			
'''
			origin = {'left_w0': -0.34399519168330406, 
			'left_w1': 0.27880100819817394, 
			'left_w2': -0.8222137023065818, 
			'left_e0': 1.7725148004015956, 
			'left_e1': 1.3863351370514427, 
			'left_s0': -0.8179952551398969, 
			'left_s1': -0.40727189918357737
			}

			goal_z = {'left_w0': -0.34399519168330406, 
			'left_w1': 0.27880100819817394, 
			'left_w2': -0.8222137023065818, 
			'left_e0': 1.7725148004015956, 
			'left_e1': 1.3863351370514427, 
			'left_s0': -0.8179952551398969, 
			'left_s1': 0.15
			}

			goal_x = {'left_w0': -0.34399519168330406, 
			'left_w1': 0.27880100819817394, 
			'left_w2': -1.5, 
			'left_e0': 1.7725148004015956, 
			'left_e1': 1.47, 
			'left_s0': -0.8179952551398969, 
			'left_s1': 0.15
			}

			goal_y = {'left_w0': -0.34399519168330406, 
			'left_w1': 0.27880100819817394, 
			'left_w2': -1.5, 
			'left_e0': 1.7725148004015956, 
			'left_e1': 1.15, 
			'left_s0': -0.5, 
			'left_s1': 0.15
			}
			# move_to_goal_pose(self.lLimb, p, self.pause_event)
			move_to_goal_joint_angle(self.lLimb, origin, self.pause_event)

			# print self.lLimb.joint_angles()
'''
			


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass


