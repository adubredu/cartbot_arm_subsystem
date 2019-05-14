#! /usr/bin/env python
import rospy
import baxter_interface
from geometry_msgs.msg import PoseStamped, Twist
from positionControl import *
from mobileTasks import *
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import tf2_ros
import tf2_geometry_msgs
import tf
import speech_recognition as sr
from math import fabs
from walk_distance_msg.msg import Walk


class microwave_tasks:
	##############   MICROWAVE OPERATIONS   #################

		def transform_microwave_marker_pose_to_robot_rf(self):
			#kinect camera axis is not the same as the robot axis so we could have
			#to perform the necessary transforms first to get both axes aligned
			#and then to transform camera rf to robot's rf
			#goal_pose is the final pose of the marker wrt the robot's rf

			#an alternative would be to directly read the transform from the marker
			#to the base but the refresh rate is about 8Hz which isn't quick enough
			#and would produce errors like lookup would require extrapolation ...
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
			return self.microwave_goal_pose

		def is_robot_close_to_kitchen(self):
			tf_buffer = tf2_ros.Buffer(rospy.Duration(3200.0))
			tf_listener = tf2_ros.TransformListener(tf_buffer)

			transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
				rospy.Duration(1.0))

			self.to_kitchen = tf2_geometry_msgs.do_transform_pose(self.dist_to_kitchen, transform)
			

		
		def open_microwave(self):
			# move_to_goal_joint_angle(self.lLimb, self.pre_open_microwave, self.pause_event)
			p = get_new_open_microwave_goal_pose(self.microwave_goal_pose)
			move_to_goal_pose(self.lLimb, p, self.pause_event)
			ang =  self.lLimb.joint_angles()
			ang['left_w1']+=1.3
			#ang['left_w0']+=0.6
			#ang['left_w1']+=0.5
			move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)
			#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			
			# rospy.signal_shutdown("moving done")

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


		def get_to_kitchen(self):
			if self.get_to_position:
				gx = self.in_kitchen_pose.pose.position.x
				gy = self.in_kitchen_pose.pose.position.y
				cx = self.to_kitchen.pose.position.x
				cy = self.to_kitchen.pose.position.y
				walk = Walk()
				if (fabs(gx - cx) > 0.05):# or (fabs(gy -cy) > 0.05):
					self.in_position = False

					# if (fabs(gy - cy) > 0.05):
					# 	if gy > cy:
					# 		walk.direction = 3
					# 		walk.distance = 3
					# 		self.walkpub.publish(walk)
					# 		time.sleep(3)
					# 		#move_right(3 steps)
					# 	elif gy < cy:
					# 		walk.direction = 4
					# 		walk.distance = 3
					# 		self.walkpub.publish(walk)
					# 		time.sleep(3)
							#move_left(3 steps)

					if (fabs(gx-cx) > 0.05):
						if gx > cx:
							walk.direction = 1
							walk.distance = gx-cx-0.2
							self.walkpub.publish(walk)
							time.sleep(3)
							#move_back(gx-cx-0.2)
						elif gx < cx:
							walk.direction = 2
							walk.distance = cx-gx-0.2
							self.walkpub.publish(walk)
							time.sleep(3)
							#move_forward(cx-gx-0.2)
						print('moving')
					self.in_position = True

				else:
					self.in_position = True





		def master_callback(self,data):

			if not data.markers:
				return
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.fridge_marker:
					self.fridge_marker_pose = data.markers[i].pose
					#self.dist_to_kitchen = data.markers[i].pose.pose.position.x
					
				if data.markers[i].id == self.kitchen_marker:
					#self.is_robot_close_to_kitchen()
					self.dist_to_kitchen = data.markers[i].pose
					# self.is_robot_close_to_kitchen()
					self.to_kitchen = self.dist_to_kitchen
					self.get_to_kitchen()
				
				if data.markers[i].id == self.microwave_marker:
					self.microwave_marker_pose = data.markers[i].pose
					#print(self.transform_microwave_marker_pose_to_robot_rf())
						
				if data.markers[i].id == self.bottle_marker:
					self.bottle_marker_pose = data.markers[i].pose
				
				if data.markers[i].id == self.bowl_marker:
					self.bowl_marker_pose = data.markers[i].pose

				if data.markers[i].id == self.shelf_object_marker:
					self.shelf_object_marker_pose = data.markers[i].pose



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
		    commands = ["open the microwave", "close the microwave"]
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


		### Microwave commands ###
		def interprete_microwave_tasks(self, command):
			if 'open' in command and 'microwave' in command:
				
				self.get_to_position = True
				#time.sleep(5)
				#self.in_position = True
				self.get_to_kitchen()
				if self.in_position:#not self.env["microwaveOpen"] and not self.env["holdingSomething"]:
					#if self.env['fridgeOpen']:
						#playPositionFile('preopenmicfromfridge.wp', self.lLimb, self.rLimb, self.pause_event)
						
					#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					#time.sleep(3)
					playPositionFile('waypoints/startpoint_microwave.wp', self.lLimb, self.rLimb, self.pause_event)
					time.sleep(2)
					self.transform_microwave_marker_pose_to_robot_rf()
					#playPositionFile('./openmicpose.wp', self.lLimb, self.rLimb, self.pause_event)
					self.open_microwave()
					# print self.microwave_goal_pose
					self.env['microwaveOpen'] = True
					self.in_position = False 
				else:
					print "Not yet in position"

			elif 'close' in command and 'microwave' in command:
				# self.gui.label_dict['command'] = 'close the microwave'
				# self.gui.update_command_display()
				self.get_to_position = True
				time.sleep(5)

				if self.in_position:#self.env["microwaveOpen"] and not self.env["holdingSomething"]:
					playPositionFile('closeMicrowave.wp', self.lLimb, self.rLimb, self.pause_event)
					self.env['microwaveOpen'] = False 
					move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
					self.in_position = False

				else:
					print "Not yet in position"


		def interprete_command(self,command):
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

			
			self.vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)


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

			self.in_kitchen_pose = PoseStamped()
			self.in_kitchen_pose.pose.position.x = 0.859
			self.in_kitchen_pose.pose.position.y = -0.0815
			self.in_kitchen_pose.pose.position.z = 0.02151

			self.walkpub = rospy.Publisher('/walk', Walk, queue_size=2)


			self.get_to_position = False
			self.in_position = False
			#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.master_callback)
			#rospy.Subscriber('/mobile_base_cam/ar_pose_marker', AlvarMarkers, self.auto_park_callback)

			self.kitchen_marker = 35
			self.dist_to_kitchen = 0
			self.dist_to_table = 0

			self.setup_speech()
			self.stopListening = self.rec.listen_in_background(self.mic, self.speech_callback, phrase_time_limit=4)
			
			#self.gui.main_loop()

			

			# print self.lLimb.endpoint_pose()
			# print self.lLimb.joint_angles()
			
			
			rospy.spin()
			
			


if __name__=="__main__":
	
	try:
		go = microwave_tasks()

	except rospy.ROSInterruptException: pass


