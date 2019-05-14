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
			m = self.lLimb.joint_angles()
			m['left_s0'] += 0.5
			m['left_s1'] += 0.2
			move_to_goal_joint_angle(self.lLimb, m, self.pause_event)
			# time.sleep(2)
			# playPositionFile('./fridge_grab_pose2.wp', self.lLimb, self.rLimb, self.pause_event)
			# time.sleep(2)

			# g = self.get_fridge_grab_goal_pose(self.bowl_marker)
			# move_to_goal_pose(self.lLimb, g, self.pause_event)
			# playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# self.fridgeOpened = True
			# rospy.signal_shutdown("moving done")




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


		
		def open_microwave(self):
			# move_to_goal_joint_angle(self.lLimb, self.pre_open_microwave, self.pause_event)
			p = get_open_microwave_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			ang =  self.lLimb.joint_angles()
			ang['left_w0']+=0.6
			ang['left_w1']+=0.5
			move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			playPositionFile('leavemic.wp', self.lLimb, self.rLimb, self.pause_event)
			

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

			

		
		def pick_bottle(self):
			
			p = get_pick_bottle_goal_pose(self.bottle_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			self.lGripper.close()
			time.sleep(1)
			playPositionFile('carrybottle.wp', self.lLimb, self.rLimb, self.pause_event)
			
			#opening motion
			# ang =  self.lLimb.joint_angles()
			# ang['left_w1']+=1.3
			# move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			# playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# self.fridgeOpened = True
			# rospy.signal_shutdown("moving done")

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

			

		
		def pick_bowl(self):
			time.sleep(3)
			p = get_pick_bowl_goal_pose(self.bowl_pose)
			# p = get_fridge_grab_goal_pose(self.bowl_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			self.lGripper.close()
			time.sleep(1)
			playPositionFile('carrybowl.wp', self.lLimb, self.rLimb, self.pause_event)
			# rospy.signal_shutdown("moving done")



		####################################
		#
		# 	COMPOUND TASKS
		#
		#######################################
		def put_food_in_microwave(self):
			playPositionFile('putFoodInMicPose.wp', self.lLimb, self.rLimb, self.pause_event)
			time.sleep(3)
			p = get_put_food_in_microwave_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			moveOnAxis(self.lLimb, 'y', .07, .02, self.pause_event)
			self.lGripper.open()
			moveOnAxis(self.lLimb, 'y', -.15, .02, self.pause_event)
			playPositionFile('closeMicrowave.wp', self.lLimb, self.rLimb, self.pause_event)


		def get_food_from_microwave(self):
			playPositionFile('putFoodInMicPose.wp', self.lLimb, self.rLimb, self.pause_event)
			time.sleep(3)
			p = get_put_food_in_microwave_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			moveOnAxis(self.lLimb, 'z', .01, .02, self.pause_event)
			moveOnAxis(self.lLimb, 'y', .05, .02, self.pause_event)
			self.lGripper.close()
			moveOnAxis(self.lLimb, 'y', -.20, .02, self.pause_event)
			playPositionFile('getOutOfMicPose.wp', self.lLimb, self.rLimb, self.pause_event)
			self.lGripper.open()


		def cook_for_seconds(self,t):
			playPositionFile('precookforseconds.wp', self.lLimb, self.rLimb, self.pause_event)
			p = get_cook_for_seconds_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			moveOnAxis(self.lLimb, 'z', -.05, .04, self.pause_event)	
			time.sleep(t)
			moveOnAxis(self.lLimb, 'z', .10, .04, self.pause_event)	
			moveOnAxis(self.lLimb, 'x', .15, .04, self.pause_event)	

		


			########################
			#					   #
			#   MASTER CALLBACK    #
			#					   #
			########################
		def master_callback(self,data):
			# print "subscriber called"
			if not data.markers:
				return
			for i in range(0,len(data.markers)):
				
				if data.markers[i].id == self.fridge_marker:
					if self.activate_open_fridge:
						found_object = True
						self.fridge_marker_pose = data.markers[i].pose
						self.transform_fridge_marker_pose_to_robot_rf()
						# print self.fridge_goal_pose
						self.open_fridge()
						self.activate_open_fridge = False
						self.activate_pick_bottle = True
						
				if data.markers[i].id == self.microwave_marker:
					if self.activate_open_microwave:
						self.microwave_marker_pose = data.markers[i].pose
						self.transform_microwave_marker_pose_to_robot_rf()
						# playPositionFile('./openmicpose.wp', self.lLimb, self.rLimb, self.pause_event)
						# print self.microwave_goal_pose
						# self.open_microwave()
						if self.cook_seconds_flag:
							self.cook_for_seconds(5)
						if self.get_food_from_microwave_flag:
							self.get_food_from_microwave()
						
						self.activate_open_microwave = False
						

				if data.markers[i].id == self.bottle_marker:
					if self.activate_pick_bottle:
						self.bottle_pose = data.markers[i].pose
						self.transform_bottle_marker_pose_to_robot_rf()
						# print self.bottle_pose
						playPositionFile('./fridge_grab_pose2.wp', self.lLimb, self.rLimb, self.pause_event)
						self.pick_bottle()
						self.activate_pick_bottle = False
						
						if self.put_water_on_table_flag:
							playPositionFile('place_bottle_on_table.wp', self.lLimb, self.rLimb, self.pause_event)
							self.lGripper.open()

						move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
						self.activate_pick_bowl = True
						# rospy.signal_shutdown("moving done")



				if data.markers[i].id == self.bowl_marker:
					if self.activate_pick_bowl:# or self.put_food_in_microwave_flag:
						self.bowl_pose = data.markers[i].pose
						self.transform_bowl_marker_pose_to_robot_rf()
						# print self.bowl_pose
						playPositionFile('./fridge_grab_pose_bowl.wp', self.lLimb, self.rLimb, self.pause_event)
						time.sleep(3)
						self.pick_bowl()
						self.activate_pick_bowl = False
						if self.put_food_in_microwave_flag:
							self.put_food_in_microwave()
							self.put_food_in_microwave_flag = False
							self.activate_open_microwave = True



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
			if not self.lGripper.calibrate():
			    print("left gripper did not calibrate")
			    sys.exit()

			self.lGripper.set_holding_force(100)
			self.lGripper.set_moving_force(100)

			self.rGripper.set_holding_force(100)
			self.rGripper.set_moving_force(100)
			# self.lGripper.open()
		
			self.head = baxter.Head()
			self.marker_pose = None
			# self.head.set_pan(0)

			self.fridgeOpened = False
			self.activate_open_fridge = True
			self.fridge_marker = 0

			self.bottleGrabbed = False
			self.activate_pick_bottle = False
			self.bottle_marker = 4

			self.microwaveOpened = False
			self.activate_open_microwave = False
			self.microwave_marker = 1


			self.bowlGrabbed = False
			self.activate_pick_bowl = False
			self.bowl_marker = 3

			self.pause_event = Event()
			
			

			self.origin = {'left_w0': -0.2834029505618302, 
			'left_w1': 1.5251603983550726, 
			'left_w2': -1.430053589506177, 
			'left_e0': 0.14802914603094242, 
			'left_e1': 0.4249126782442596, 
			'left_s0': 0.7804127258367043, 
			'left_s1': -0.6975777632908919
			}

			self.pre_open_microwave = {'left_w0': 1.2666846355963803, 
			'left_w1': 0.7225049510940299, 
			'left_w2': -1.6095293416887704, 
			'left_e0': 0.8536603084582327, 
			'left_e1': 1.2950632801722606, 
			'left_s0': -0.7148350471546028, 
			'left_s1': -1.0607477148227635
			} 

			move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			

			#TASK FLAGS
			self.put_food_in_microwave_flag = True
			self.put_water_on_table_flag = False 
			self.cook_seconds_flag = True 
			self.get_food_from_microwave_flag = True 



			# print self.lLimb.endpoint_pose()
			# print self.lLimb.joint_angles()
			rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.master_callback)
			
			rospy.spin()
			
			


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass


