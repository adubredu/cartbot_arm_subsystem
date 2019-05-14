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
			if not once:
				once = True
				self.place = 11
				if self.place != 0:
					self.move_to_position(data)

				else:
					self.move_to_origin()
		


		def move_to_position(self, data):

			#initializing moveit node shenanigans
			moveit_commander.roscpp_initialize(sys.argv)
			robot = moveit_commander.RobotCommander()
			scene = moveit_commander.PlanningSceneInterface()
			left_arm = moveit_commander.MoveGroupCommander('left_arm')
			left_arm.set_planner_id('RRTConnectkConfigDefault')
			left_arm.set_planning_time(10)

			found_object = False
		
			for i in range(0,len(data.markers)):
				#detected markers are in an array
				#Here, we search for the marker with ID stored in variable 
				#self.place
				if data.markers[i].id == self.place:
					found_object = True
					marker_pose = data.markers[i].pose
					

					#Here, we transform the pose of the marker to the reference frame 'base'
					#which is the reference frame of the entire robot and from which
					#all other poses are relative to
					tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
					tf_listener = tf2_ros.TransformListener(tf_buffer)

					transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
						rospy.Duration(1.0))
					goal_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)
					
					#setting some offsets to the goal position for our convenience
					# goal_pose.pose.position.z -= 0.10
					print goal_pose
					goal_pose.pose.position.y -= 0.90


					gripper_pose = self.limb.endpoint_pose()

					goal_pose.pose.orientation.x = gripper_pose['orientation'][0]
					goal_pose.pose.orientation.y = gripper_pose['orientation'][1]
					goal_pose.pose.orientation.z = gripper_pose['orientation'][2]
					goal_pose.pose.orientation.w = gripper_pose['orientation'][3]
					
					#passing the pose goal into the moveit motion planner to plan the trajectory
					left_arm.set_pose_target(goal_pose)
					left_arm.set_start_state_to_current_state()
					left_plan = left_arm.plan()
					print "done planning"

					#executing planned trajectory
					rospy.sleep(5)
					left_arm.execute(left_plan)
					rospy.sleep(5)
					print self.limb.endpoint_pose()
    				
			if not found_object:
				print "Could not find marker ID "+ str(self.place)





		def move_to_origin(self):
			lLimb = baxter.Limb('left')
			rLimb = baxter.Limb('right')
			fPath = '/'
			pause_event = None
			playPositionFile(fPath, lLimb, rLimb, pause_event)
			rospy.sleep(3)

		def __init__(self):
			rospy.init_node('gotomarker')
			print "What can I do for you"
			self.limb = baxter.Limb('left')
			self.place = 0
			sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback)
			rospy.spin()


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass


'''
******************************************************************************************
*************************Voice Recognition Stuff [-_-]************************************
*************************Sammy knows the way!*********************************************
******************************************************************************************

		def start_listen(self):
			r = sr.Recognizer()
			m = sr.Microphone()
			r.pause_threshold = .5
			r.dynamic_energy_threshold = False

			with m as source:
				r.adjust_for_ambient_noise(source, 1)

			stopListening = r.listen_in_background(m, self.heard, phrase_time_limit=4)


		def interprete_command(self, rawCommand):
			# r = sr.Recognizer()
			# with sr.Microphone() as source:
			# 	audio = r.listen(source)
			# try:
			# 	print(r.recognize_google(audio))
			if ('fridge' in rawCommand):
				self.place = 3
			elif 'microwave' in rawCommand:
				self.place = 5
			elif 'origin' in rawCommand:
				self.place = 0
			# except sr.UnknownValueError:
			# 	print "Google gould not understand audio"

			# except sr.RequestError as e:
			# 	print("Could not request from Google; {0}".format(e))
			#self.place = 4


		# callback function when audio data is obtained
		def heard(self, recognizer, audio):
			# Defining Commands to be accepted
			credsJson = ""
			with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
				credsJson = gspeechcreds.read()
			
			sens = 1
			commands = ["fridge", "microwave", "origin"]
			print('trying to recognize')
			try:
				commandIter = [command[0] for command in commands]
				rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', credentials_json=credsJson, preferred_phrases=commands)
				print(rawCommand)
				self.interprete_command(rawCommand)
			except sr.UnknownValueError:
				print("could not understand audio")
				pass
			except sr.RequestError as e:
				print("Recognition error; {}".format(e))
'''