#! /usr/bin/env python
import rospy
import sys
import baxter_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from positionControl import *
from mobileTasks import *
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
import tf
import speech_recognition as sr
from robot_gui import robot_gui
from geometry_msgs.msg import Twist

once = False

class gotomarker:

		#######   MOBILE BASE OPERATIONS  #############

		def imu_callback(self, imu_data):
			orr = (pi /180)*imu_data.data
			orrientation = atan2(sin(orr),cos(orr))
			self.orientation = orrientation
			if not self.init_gotten:
				self.init_orientation = orrientation
				self.init_gotten = True


		def turn(self,angle):
			rate = rospy.Rate(3)
			diff = fabs(self.orientation - self.init_orientation)
			while  not (angle < diff):# < 1.658):
				velocity = Twist()
				velocity.angular.z = -1.0
				self.vel_pub.publish(velocity)
				rospy.Subscriber("/imu_heading", Float32, self.imu_callback)
				print diff
				diff = fabs(self.orientation - self.init_orientation)

				if rospy.is_shutdown():
					break
				rate.sleep()

			self.turn_complete = True
			velocity = Twist()
			velocity.angular.z = 0.0
			self.vel_pub.publish(velocity)
			print "Done turning 90 degrees"
			self.init_orientation = 0.0
			self.orientation = 0.0
			self.init_gotten = False



		def approach36(self, dist, data):
			if not self.donewith36:
				z_dist_1 = data.pose.pose.position.z
				x_dist_1 = data.pose.pose.position.x

				if ((dist-0.01)>z_dist_1) or (z_dist_1 > (dist+0.01)):
					velocity = Twist()

					#0.05 in order to prevent arm from hitting table
					if z_dist_1 < (dist-0.01):
						velocity.linear.x = -1.0
					elif z_dist_1 > (dist+0.01):
						velocity.linear.x = 1.0

					self.vel_pub.publish(velocity)
					

					if z_dist_1 > 2.0:
						time.sleep(0.5)
					else:
						time.sleep(0.2)
					

					if x_dist_1 < -0.25:
						vel = Twist()
						vel.angular.z = 1.0
						self.vel_pub.publish(vel)
						print "left"
					elif x_dist_1 > 0.25:
						vel = Twist()
						vel.angular.z = -1.0
						self.vel_pub.publish(vel)
						print "right"
					x=Twist()
					self.vel_pub.publish(x)
					time.sleep(0.1)
					print z_dist_1

				else:
					v = Twist()
					self.vel_pub.publish(v)
					time.sleep(3)
					self.donewith36 = True
					self.turn(2.8)
					v=Twist()
					v.linear.x = -1.0
					self.vel_pub.publish(v)
					time.sleep(3)



		def approach5(self, dist, data):
			if not self.donewith5:
				z_dist_5 = data.pose.pose.position.z
				x_dist_5 = data.pose.pose.position.x

				if ((dist-0.02)>z_dist_5) or (z_dist_5 > (dist+0.02) or (x_dist_5 < -0.06) or (x_dist_5 > 0.06)):
					velocity = Twist()

					if z_dist_5 < (dist-0.02):
						velocity.linear.x = -1.0
					elif z_dist_5 >(dist+0.02):
						velocity.linear.x = 1.0

					self.vel_pub.publish(velocity)
					time.sleep(0.4)
				
					if x_dist_5 < -0.07:
						vel = Twist()
						vel.angular.z = 1.0
						self.vel_pub.publish(vel)
						print "left"
					elif x_dist_5 > 0.07:
						vel = Twist()
						vel.angular.z = -1.0
						self.vel_pub.publish(vel)
						print "right"
					time.sleep(0.2)
					
				else:
					v = Twist()
					self.vel_pub.publish(v)
					time.sleep(3)
					self.donewith5 = True
					print 'done localizing'
					self.activate_auto_park = False
				


		def auto_park_callback(self, data):
			if self.activate_auto_park:
				if not data.markers:
					#print "didn't update"
					#v=Twist()
					#self.vel_pub.publish(v)
					return

				#for i in range(0, len(data.markers)):
					# if data.markers[i].id == 34:
						# if not self.donewith36:
							#self.approach36(1.6, data.markers[i])


					#if data.markers[i].id == 5:
						#if not self.donewith5:
							#self.approach5(0.75, data.markers[i])
					#	self.dist_to_table = data.markers[i].pose.pose.position.z
					#	self.env['distanceToKitchen'] = self.dist_to_kitchen
		    		#		self.env['distanceToTable'] = self.dist_to_table
					#	self.gui.update_labels(self.env)
		        	#		self.gui.update_command_display()


		#######   FRIDGE OPERATIONS   ##############
		
		#Here, we transform the pose of the marker to the reference frame 'base'
		#which is the reference frame of the entire robot and from which
		#all other poses are relative to
		def transform_fridge_marker_pose_to_robot_rf(self):
			#kinect camera axis not the same as the robot axis so we could have
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
			#m = self.lLimb.joint_angles()
			#m['left_s0'] += 0.5
			#m['left_s1'] += 0.2
			move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
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
			ang['left_w1']+=1.3
			#ang['left_w0']+=0.6
			#ang['left_w1']+=0.5
			move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			#playPositionFile('leavemic.wp', self.lLimb, self.rLimb, self.pause_event)
			

			#move to origin 
			# move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			# ang =  self.lLimb.joint_angles()
			# ang['left_w1']+=0.8
			# move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			# # playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# # self.fridgeOpened = True
			# rospy.signal_shutdown("moving done")






		#################   BOTTLE OPERATIONS   ######################

		def transform_bottle_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.bottle_marker_pose.pose.position.y
			marker_pose.pose.position.x = self.bottle_marker_pose.pose.position.x
			marker_pose.pose.position.z = self.bottle_marker_pose.pose.position.z
			marker_pose.pose.orientation = self.bottle_marker_pose.pose.orientation

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
			playPositionFile('carrybottle2.wp', self.lLimb, self.rLimb, self.pause_event)
			
			#opening motion
			# ang =  self.lLimb.joint_angles()
			# ang['left_w1']+=1.3
			# move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			# playPositionFile('./openFreezer.wp', self.lLimb, self.rLimb, self.pause_event)
			# self.fridgeOpened = True
			# rospy.signal_shutdown("moving done")
	





#################   SHELF OBJECT OPERATIONS   ######################

		def transform_shelf_object_marker_pose_to_robot_rf(self):
			#kinect camera axi is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf
			marker_pose = PoseStamped()
			marker_pose.pose.position.y = self.shelf_object_marker_pose.pose.position.y
			marker_pose.pose.position.x = self.shelf_object_marker_pose.pose.position.x
			marker_pose.pose.position.z = self.shelf_object_marker_pose.pose.position.z
			marker_pose.pose.orientation = self.shelf_object_marker_pose.pose.orientation

			tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))
			self.shelf_object_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)

			

		
		def pick_shelf_object(self):
			
			self.transform_shelf_object_marker_pose_to_robot_rf()
			#print self.shelf_object_pose
			playPositionFile('./startshelfpicking.wp', self.lLimb, self.rLimb, self.pause_event)
			self.lGripper.open()
			p = get_take_from_shelf_goal_pose(self.shelf_object_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			self.lGripper.close()
			#time.sleep(1)
			#playPositionFile('carrybottle2.wp', self.lLimb, self.rLimb, self.pause_event)





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
			marker_pose.pose.position.y = self.bowl_marker_pose.pose.position.y
			marker_pose.pose.position.x = self.bowl_marker_pose.pose.position.x
			marker_pose.pose.position.z = self.bowl_marker_pose.pose.position.z
			marker_pose.pose.orientation = self.bowl_marker_pose.pose.orientation

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
			self.transform_microwave_marker_pose_to_robot_rf()
			p = get_put_food_in_microwave_goal_pose(self.microwave_goal_pose)
			moveOnAxis(self.lLimb, 'x', .02, .02, self.pause_event)
			time.sleep(2)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			
			time.sleep(2)
			moveOnAxis(self.lLimb, 'y', .04, .02, self.pause_event)
			time.sleep(2)
			moveOnAxis(self.lLimb, 'y', .02, .02, self.pause_event)
			time.sleep(1)
			self.lGripper.open()
			moveOnAxis(self.lLimb, 'z', -.02, .02, self.pause_event)
			time.sleep(1)
			moveOnAxis(self.lLimb, 'y', -.15, .02, self.pause_event)
			# playPositionFile('closeMicrowave.wp', self.lLimb, self.rLimb, self.pause_event)
			playPositionFile('getOutOfMicPose.wp', self.lLimb, self.rLimb, self.pause_event)
			self.lGripper.open()


		def get_food_from_microwave(self):
			playPositionFile('putFoodInMicPose2.wp', self.lLimb, self.rLimb, self.pause_event)
			playPositionFile('twistarm.wp', self.lLimb, self.rLimb, self.pause_event)
			time.sleep(3)
			p = get_get_food_in_microwave_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			# moveOnAxis(self.lLimb, 'z', -.02, .02, self.pause_event)
			# time.sleep(2)
			moveOnAxis(self.lLimb, 'z', -.04, .02, self.pause_event)
			moveOnAxis(self.lLimb, 'y', .05, .02, self.pause_event)
			self.lGripper.close()
			moveOnAxis(self.lLimb, 'y', -.25, .02, self.pause_event)
			playPositionFile('getOutOfMicPose.wp', self.lLimb, self.rLimb, self.pause_event)
			self.lGripper.open()


		def cook_for_seconds(self,t):
			playPositionFile('precookforseconds.wp', self.lLimb, self.rLimb, self.pause_event)
			p = get_cook_for_seconds_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			moveOnAxis(self.lLimb, 'z', -.05, .04, self.pause_event)
			self.env['microwaveOn'] = True	
			time.sleep(t)
			moveOnAxis(self.lLimb, 'z', .10, .04, self.pause_event)	
			moveOnAxis(self.lLimb, 'x', .15, .04, self.pause_event)
			self.env['microwaveOn'] = False	

		


			########################
			#					   #
			#   MASTER CALLBACK    #
			#					   #
			########################
		def master_callback(self,data):
			# print "subscriber called"
			# self.gui.update_labels(self.env)
			# self.gui.update_root()

			if not data.markers:
				return
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.fridge_marker:
					self.fridge_marker_pose = data.markers[i].pose
					self.dist_to_kitchen = data.markers[i].pose.pose.position.x
					#if not self.worked:
						#self.transform_fridge_marker_pose_to_robot_rf()
						#self.open_fridge()
						#self.worked = True

				
				if data.markers[i].id == self.microwave_marker:
					self.microwave_marker_pose = data.markers[i].pose
					#self.transform_microwave_marker_pose_to_robot_rf()
					'''
					if not self.worked:
						self.transform_microwave_marker_pose_to_robot_rf()
						self.open_microwave()
						self.worked = True
						'''
						
				if data.markers[i].id == self.bottle_marker:
					self.bottle_marker_pose = data.markers[i].pose
				
				if data.markers[i].id == self.bowl_marker:
					self.bowl_marker_pose = data.markers[i].pose
					#self.transform_bowl_marker_pose_to_robot_rf()
					#print self.bowl_pose
					

				if data.markers[i].id == self.shelf_object_marker:
					self.shelf_object_marker_pose = data.markers[i].pose
					#self.pick_shelf_object()
					#if not self.worked:
						#self.pick_shelf_object()
						#self.worked = True
					
						


		def valmap(self,value, istart, istop, ostart, ostop):
  			return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))




  		####################################
  		# SPEECH RECOGNITION METHODS	####
  		####################################

  		def setup_speech(self):
		    print "in speech setup"
		    self.rec.pause_threshold = .5
		    self.rec.dynamic_energy_threshold = False
		    with self.mic as source:
		        self.rec.adjust_for_ambient_noise(source, 3)


		def speech_callback(self, recognizer, audio):
		    # Defining Commands to be accepted
		    global t2s, dialog
		    #credsJson = ""
		    #with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
		        #credsJson = gspeechcreds.read()
		    
		    sens = 1
		    commands = ["take","close gripper", "open gripper", "stop", "stop", "stop", "open the microwave",
		                "faster", "slower", "open the fridge", "move to zero", "get a water bottle", "fridge", 
		                "place on", "table", "fridge is open", "holding something", "fridge is closed", "close the microwave","start the microwave",
		                "hand is empty", "put food in the microwave", "get food from the microwave", "start", "turn off", "continue", "cook for",
		                "put", "food", "is open", "get the food", "activate auto localization","activate mobile base",
		                "robot is localized","move arm lower","move arm higher", "put water bottle on the table", "ground grab mode", "table grab mode",
		                "pick object from floor", "forward", "backward", "left", "right", "pause", "drop object", "drop in bin"]
		    dialog = "Listening..."
		    print("listening")
		    #print('Distance to kitchen: ',self.dist_to_kitchen)
		    #print('Distance to table: ',self.dist_to_table)
		    #self.env['distanceToKitchen'] = self.dist_to_kitchen
		    #self.env['distanceToTable'] = self.dist_to_table
		    try:
		        commandIter = [command[0] for command in commands]
		        global rawCommand
		        rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', preferred_phrases=commands)
		        dialog = rawCommand
		        print("understood")
		        print(dialog)
		        self.interprete_command(rawCommand)

		        #self.gui.label_dict['dialog'] = dialog
		        #self.gui.update_labels(self.env)
		        #self.gui.update_command_display()

		    except sr.UnknownValueError:
		        dialog = "Listening..."
		        pass
		    except sr.RequestError as e:
		        print("Recognition error; {}".format(e))





		##FRIDGE TASKS
		def interprete_fridge_tasks(self, command):
			
			if 'open' in command and 'the fridge' in command:
				# self.gui.label_dict['command'] = 'open the fridge'
				# self.gui.update_command_display()

				if True:#not self.env['fridgeOpen']:
					self.transform_fridge_marker_pose_to_robot_rf()
					self.open_fridge()
					self.env['fridgeOpen'] = True
				else:
					print "Fridge is already open"

			# Get waterbottle from fridge #
			elif 'get' in command and 'water bottle' in command:
				# self.gui.label_dict['command'] = 'get water bottle'
				# self.gui.update_command_display()

				if True:#self.env['fridgeOpen'] and not self.env['hasBottle']:
					self.lGripper.open()
					self.transform_bottle_marker_pose_to_robot_rf()
					playPositionFile('./fridge_grab_pose3.wp', self.lLimb, self.rLimb, self.pause_event)
					# print self.bottle_pose
					self.pick_bottle()
					self.env['hasBottle'] = True
					self.env['holdingSomething'] = True
					self.env['bottleInFridge'] = False
				else:
					print "Either fridge is closed or robot already has the bottle"
		        

			# Close the fridge #
			elif 'close' in command and 'the fridge' in command:
				# self.gui.label_dict['command'] = 'close the fridge'
				# self.gui.update_command_display()

				playPositionFile('closeTheFridge.wp', self.lLimb, self.rLimb, self.pause_event)
				move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
				self.env['fridgeOpen'] = False
				  
		    

		    # Place water bottle on table #
			elif 'place on the table' in command:
				# self.gui.label_dict['command'] = 'place on the table'
				# self.gui.update_command_display()

				if self.env['hasBottle']:
					playPositionFile('place_bottle_on_table.wp', self.lLimb, self.rLimb, self.pause_event)
					self.lGripper.open()
					playPositionFile('fromTableToOrigin.wp', self.lLimb, self.rLimb, self.pause_event)
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					self.env['bottleOnTable'] = True
					self.env['holdingSomething'] = False
					self.env['hasBottle'] = False
		        
				else:
					print "Robot doesn't have the bottle"


			# Full command - Put water bottle on table #
			elif 'put' in command and 'water bottle' in command and 'table' in command:
				# self.gui.label_dict['command'] = 'put water bottle on table'
				# self.gui.update_command_display()

				self.transform_fridge_marker_pose_to_robot_rf()
				self.open_fridge()
				self.env['fridgeOpen'] = True

				self.transform_bottle_marker_pose_to_robot_rf()
				playPositionFile('./fridge_grab_pose3.wp', self.lLimb, self.rLimb, self.pause_event)
				self.pick_bottle()
				self.env['hasBottle'] = True
				self.env['holdingSomething'] = True

				playPositionFile('place_bottle_on_table.wp', self.lLimb, self.rLimb, self.pause_event)
				self.lGripper.open()
				playPositionFile('fromTableToOrigin.wp', self.lLimb, self.rLimb, self.pause_event)
				move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
				self.env['bottleOnTable'] = True
				self.env['holdingSomething'] = False

				playPositionFile('closeTheFridge.wp', self.lLimb, self.rLimb, self.pause_event)
				move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
				self.env['fridgeOpen'] = False
				self.env['hasBottle'] = False
			#### End Fridge task commands ####






		### Microwave commands ###
		def interprete_microwave_tasks(self, command):
			if 'open' in command and 'the microwave' in command:
				# self.gui.label_dict['command'] = 'open the microwave'
				# self.gui.update_command_display()

				if True:#not self.env["microwaveOpen"] and not self.env["holdingSomething"]:
					#if self.env['fridgeOpen']:
						#playPositionFile('preopenmicfromfridge.wp', self.lLimb, self.rLimb, self.pause_event)
						
					#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					#time.sleep(3)
					self.transform_microwave_marker_pose_to_robot_rf()
					#playPositionFile('./openmicpose.wp', self.lLimb, self.rLimb, self.pause_event)
					self.open_microwave()
					# print self.microwave_goal_pose
					self.env['microwaveOpen'] = True 
				else:
					print "Either there is something in gripper or Microwave is already opened"

			elif 'close' in command and 'microwave' in command:
				# self.gui.label_dict['command'] = 'close the microwave'
				# self.gui.update_command_display()

				if True:#self.env["microwaveOpen"] and not self.env["holdingSomething"]:
					playPositionFile('closeMicrowave.wp', self.lLimb, self.rLimb, self.pause_event)
					self.env['microwaveOpen'] = False 
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)

				else:
					print "Either there is something in gripper or Microwave is already closed"


			elif 'start' in command and 'microwave' in command:
				# self.gui.label_dict['command'] = 'start the microwave'
				# self.gui.update_command_display()

				if True:#(not self.env["microwaveOpen"]) and (not self.env["holdingSomething"]) and (not self.env["microwaveOn"]):
					playPositionFile('precookforseconds.wp', self.lLimb, self.rLimb, self.pause_event)
					p = get_cook_for_seconds_goal_pose(self.microwave_goal_pose)
					move_to_goal_pose(self.lLimb, p, self.pause_event)
					moveOnAxis(self.lLimb, 'z', -.05, .04, self.pause_event)	
					self.env['microwaveOn'] = True

				else:
					print "Either microwave is open or robot is holding something or microwave is already on"


			elif ('turn off' in command) or ('stop'in command and 'microwave' in command):
				# self.gui.label_dict['command'] = 'stop the microwave'
				# self.gui.update_command_display()

				if self.env["microwaveOn"] and (not self.env["holdingSomething"]):
					moveOnAxis(self.lLimb, 'z', .10, .04, self.pause_event)	
					moveOnAxis(self.lLimb, 'x', .15, .04, self.pause_event)	
					self.env['microwaveOn'] = False
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)

				else: 
					print "Either microwave is off or robot is holding something"


			elif 'cook' in command and 'seconds' in command:
				# self.gui.label_dict['command'] = 'cook for seconds'
				# self.gui.update_command_display()

				t = [int(s) for s in command.split() if s.isdigit()]
				print(t)
				if not t:
					print("No time given")
				elif (not self.env["microwaveOpen"]) and (not self.env["holdingSomething"]):
					self.env['microwaveOn'] = True
					self.cook_for_seconds(t[0])
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					self.env['microwaveOn'] = False

			elif 'put' in command and 'microwave' in command:
				# self.gui.label_dict['command'] = 'put food in the microwave'
				# self.gui.update_command_display()

				#if nothing is opened
				if not self.env["fridgeOpen"] and not self.env["microwaveOpen"] and not self.env["holdingSomething"] and not self.env["foodInMicrowave"]:
					#open microwave
					print "opening microwave"
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					self.transform_microwave_marker_pose_to_robot_rf()
					playPositionFile('./openmicpose.wp', self.lLimb, self.rLimb, self.pause_event)
					self.open_microwave()
					self.env['microwaveOpen'] = True 
					print "done opening microwave"

					#open fridge
					print "opening fridge"
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					self.transform_fridge_marker_pose_to_robot_rf()
					self.open_fridge()
					self.env['fridgeOpen'] = True
					print "done opening fridge"

					#pick bowl from fridge
					print "picking bowl from fridge"
					self.transform_bowl_marker_pose_to_robot_rf()
					playPositionFile('./fridge_grab_pose_bowl.wp', self.lLimb, self.rLimb, self.pause_event)
					time.sleep(3)
					self.pick_bowl()
					print "done picking bowl from fridge"
					self.env['holdingSomething'] = True

					#put bowl in microwave and close microwave
					print "putting food in microwave"
					self.put_food_in_microwave()
					self.env['foodInMicrowave'] = True
					self.env['holdingSomething'] = False
					self.env['foodInFridge'] = False

					#close microwave
					playPositionFile('closeMicrowave.wp', self.lLimb, self.rLimb, self.pause_event)
					self.env['microwaveOpen'] = False 
					
					#close fridge
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					playPositionFile('closeTheFridge.wp', self.lLimb, self.rLimb, self.pause_event)
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					self.env['fridgeOpen'] = False

				#if fridge is already open
				elif self.env["fridgeOpen"] and not self.env["microwaveOpen"] and not self.env["holdingSomething"] and not self.env["foodInMicrowave"]:
					#open microwave
					self.transform_microwave_marker_pose_to_robot_rf()
					playPositionFile('./openmicpose.wp', self.lLimb, self.rLimb, self.pause_event)
					self.open_microwave()
					self.env['microwaveOpen'] = True 

					#pick bowl from fridge
					self.transform_bowl_marker_pose_to_robot_rf()
					playPositionFile('./fridge_grab_pose_bowl.wp', self.lLimb, self.rLimb, self.pause_event)
					time.sleep(3)
					self.pick_bowl()

					#put bowl in microwave and close microwave
					self.put_food_in_microwave()
					self.env['foodInMicrowave'] = True
					#close fridge
					print "Close fridge not implemented yet"


				#if microwave is  open but fridge is closed
				elif not self.env["fridgeOpen"] and self.env["microwaveOpen"] and not self.env["holdingSomething"] and not self.env["foodInMicrowave"]:
					#open fridge
					self.transform_fridge_marker_pose_to_robot_rf()
					self.open_fridge()
					self.env['fridgeOpen'] = True

					#pick bowl from fridge
					self.transform_bowl_marker_pose_to_robot_rf()
					playPositionFile('./fridge_grab_pose_bowl.wp', self.lLimb, self.rLimb, self.pause_event)
					time.sleep(3)
					self.pick_bowl()

					#put bowl in microwave and close microwave
					self.put_food_in_microwave()
					self.env['foodInMicrowave'] = True
					#close fridge
					print "Close fridge not implemented yet"

				#if both fridge and microwave are open
				elif self.env["fridgeOpen"] and self.env["microwaveOpen"] and not self.env["holdingSomething"] and not self.env["foodInMicrowave"]:
					#pick bowl from fridge
					self.transform_bowl_marker_pose_to_robot_rf()
					playPositionFile('./fridge_grab_pose_bowl.wp', self.lLimb, self.rLimb, self.pause_event)
					time.sleep(3)
					self.pick_bowl()
					self.env['foodInMicrowave'] = True
					#put bowl in microwave and close microwave
					self.put_food_in_microwave()


			elif 'get' in command and 'microwave' in command:
				# self.gui.label_dict['command'] = 'get food from the microwave'
				# self.gui.update_command_display()
				if True:#not self.env["microwaveOpen"] and self.env["foodInMicrowave"]:
					#open microwave
					
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					time.sleep(2)
					self.transform_microwave_marker_pose_to_robot_rf()
					#playPositionFile('./openmicpose.wp', self.lLimb, self.rLimb, self.pause_event)
					self.open_microwave()
					self.env['microwaveOpen'] = True 
					time.sleep(2)
					self.get_food_from_microwave()
					self.env['foodInMicrowave'] = False
		        
				else:
					print "There's no food in the microwave"
			# end microwave Commands
			### End task related commands ###



		def reset_environment(self):
			self.env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 
				'bottleInFridge': True, 'microwaveOpen': False, 'holdingSomething': False, 
				'microwaveOn': False, 'foodInMicrowave': False, 'foodInFridge': True, 
				'foodOnTable': False, 'robotlocalized': True, 'mobileBaseActivated': False,
				'distanceToKitchen' : self.dist_to_kitchen, 'distanceToTable' : self.dist_to_table})



		def set_environment_state(self, command):
			if 'fridge is open' in command:
				 self.env['fridgeOpen'] = True
		        
			elif "fridge is closed" in command:
				self.env['fridgeOpen'] = False
		        
			elif "holding something" in command:
				self.env['hasBottle'] = True
				self.env['holdingSomething'] = True
		        
			elif "hand is empty" in command:
				self.lGripper.open()
				self.env['hasBottle'] = False
				self.env['holdingSomething'] = True
		        
			elif "microwave is open" in command:
				self.env['microwaveOpen'] = True
		        
			elif "microwave is closed" in command:
				self.env['microwaveOpen'] = False
		        
			elif 'microwave is on' in command:
				self.env['microwaveOn'] = True
		        
			elif 'microwave is off' in command:
				self.env['microwaveOn'] = False 
		        
			elif 'food in microwave' in command:
				self.env['foodInMicrowave'] = True
				self.env['foodInFridge'] = False
				self.env['foodOnTable'] = False
		        
			elif 'microwave is empty' in command:
				self.env['foodInMicrowave'] = False
		        
			elif 'robot is localized' in command:
				self.env['robotlocalized'] = True
		        
			elif 'food in fridge' in command:
				self.env['foodInFridge'] = True
				
			elif 'food on table' in command:
				self.env['foodOnTable'] = True
				self.env['foodInFridge'] = False
				self.env['foodInMicrowave'] = False

			elif "don't move" in command:
				self.activate_auto_park = False
				




		def miscellaneous_tasks(self, command):
			if 'open gripper' in command:
				self.lGripper.open()

			elif 'close gripper' in command:
				self.lGripper.close()

			elif 'reset environment' in command:
				self.reset_environment()

			elif 'move to origin' in command:
				move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			elif 'pick up object' in command:
				
				pickFromFloor(self.lLimb, self.rLimb,self.lGripper, self.pause_event)
			elif 'drop object' in command:
				dropObject(self.lLimb, self.rLimb, self.lGripper, self.pause_event)
			
	
			elif 'take' in command and 'shelf' in command.lower():			
				self.lGripper.open()
				self.pick_shelf_object()

			elif "let's go" in command:
				self.head.set_pan(0)
				playPositionFile('movearmtohead.wp', self.lLimb, self.rLimb, self.pause_event)
				
			elif "you are in the kitchen" in command:
				#self.head.set_pan(1.57)
				playPositionFile('kitchengrab.wp', self.lLimb, self.rLimb, self.pause_event)
				self.env['robotlocalized'] = True
				move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)

			elif 'go' in command and 'kitchen' in command:
				self.activate_auto_park = True

			elif "adjusting" in command:
				
				playPositionFile('adjusting.wp', self.lLimb, self.rLimb, self.pause_event)
				self.head.set_pan(1.57)

			elif "drop in bin" in command:
				
				playPositionFile('putInBin.wp', self.lLimb, self.rLimb, self.pause_event)
				self.lGripper.open()
				playPositionFile('finishDropInBin.wp', self.lLimb, self.rLimb, self.pause_event)
				



		######################################
		##   VOICE    COMMAND INTERPRETATION
		######################################
		def interprete_command(self,command):
			
			self.set_environment_state(command)
			self.miscellaneous_tasks(command)
			self.interprete_fridge_tasks(command)
			self.interprete_microwave_tasks(command)

		
		    

		def __init__(self):
			rospy.init_node('vision_move', disable_signals=True)
			self.limb = baxter.Limb('left')
			self.place = 0
			self.baxter_enabler = baxter.RobotEnable(versioned=True)
			self.baxter_enabler.enable()

			self.lLimb = baxter.Limb('left')
			self.rLimb = baxter.Limb('right')
			self.lGripper = baxter.Gripper('left')
			self.rGripper = baxter.Gripper('right')
			#print self.lLimb.endpoint_pose()

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
			#self.head.set_pan(1.57)

			#mobile base attributes
			self.init_orientation = 0.0
			self.orientation = 0.0
			self.donewith36 = False
			self.donewith5 = False
			self.activate_auto_park = False
			self.vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

			#gui 
			self.gui = robot_gui()

			#Speech recognition variables
			self.rec = sr.Recognizer()
			self.mic = sr.Microphone()

			#Object Marker IDs
			self.fridge_marker = 0
			self.bottle_marker = 4
			self.microwave_marker = 1
			self.bowl_marker = 3
			self.shelf_object_marker = 6

			self.pause_event = Event()
			self.worked = False

			#Environment variables
			self.env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 
				'bottleInFridge': True, 'microwaveOpen': False, 'holdingSomething': False, 
				'microwaveOn': False, 'foodInMicrowave': False, 'foodInFridge': True, 
				'foodOnTable': False, 'robotlocalized': False, 'mobileBaseActivated': False,
				'distanceToKitchen' : 0.0, 'distanceToTable' : 0.0})

			#Origin joint angles
			self.origin = {'left_w0': -0.07286408742455715, 
			'left_w1': 1.180398216277826, 
			'left_w2': -1.7725148004015956, 
			'left_e0': 0.17257283863710904, 
			'left_e1': 0.05138835639416136, 
			'left_s0': 0.6879903833666081, 
			'left_s1': -0.5414952181235511
			}
			self.activate_auto_park = True
			#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.master_callback)
			#rospy.Subscriber('/mobile_base_cam/ar_pose_marker', AlvarMarkers, self.auto_park_callback)

			self.dist_to_kitchen = 0
			self.dist_to_table = 0

			self.setup_speech()
			self.stopListening = self.rec.listen_in_background(self.mic, self.speech_callback, phrase_time_limit=4)
			
			#self.gui.main_loop()

			

			#print self.lLimb.endpoint_pose()
			#print self.lLimb.joint_angles()
			
			
			rospy.spin()
			
			


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass


'''
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
'''
