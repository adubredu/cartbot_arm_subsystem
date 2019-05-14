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
from object_recognition_msgs.msg import RecognizedObjectArray

once = False

class grab_object:
		def object_callback(self, data):
			for i in range(0, len(data.objects)):
				if data.objects[i].type.key == self.coke_key:
					self.object_index = i
					self.confidence = data.objects[i].confidence

			if self.object_index >= 0:
				self.object_pose.pose = data.objects[self.object_index].pose.pose.pose

			else:
				print("Could not find object")
				return


		def move_to_position(self, object_pose):
			#initializing moveit node shenanigans
			moveit_commander.roscpp_initialize(sys.argv)
			robot = moveit_commander.RobotCommander()
			scene = moveit_commander.PlanningSceneInterface()
			left_arm = moveit_commander.MoveGroupCommander('left_arm')
			# left_arm.set_planner_id('RRTConnectkConfigDefault')
			# left_arm.set_planning_time(20)
			
			print object_pose.pose
			if object_pose.pose.position.z == 0:
				trans_pose = PoseStamped()
				trans_pose.pose.position.x = object_pose.pose.position.y 
				trans_pose.pose.position.y = object_pose.pose.position.x  
				trans_pose.pose.position.z = object_pose.pose.position.z  
				
				
				
				#Here, we transform the pose of the marker to the reference frame 'base'
				#which is the reference frame of the entire robot and from which
				#all other poses are relative to
				# tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
				# tf_listener = tf2_ros.TransformListener(tf_buffer)
				
				# transform = tf_buffer.lookup_transform('base', 'camera_depth_optical_frame',rospy.Time(0),
				# 	rospy.Duration(1.0))
				#goal_pose = tf2_geometry_msgs.do_transform_pose(trans_pose, transform)
				goal_pose = PoseStamped()
				# goal_pose.pose.position.x = object_pose.pose.position.y + 0.3
				# goal_pose.pose.position.y = object_pose.pose.position.x * -1
				# goal_pose.pose.position.z = transform.transform.translation.z - object_pose.pose.position.z
				
				#setting some offsets to the goal position for our convenience
				# goal_pose.pose.position.z += 0.08
				print "***************Goal Pose Calculated************************"
				
				# print "limb pose"
				# print self.limb.endpoint_pose()
				

				
				#goal_pose.pose.position.y -= 0.90


				gripper_pose = self.limb.endpoint_pose()
				goal_pose.pose.position.x = gripper_pose['position'][0]#0.63
				goal_pose.pose.position.y = gripper_pose['position'][1]#0.19519
				goal_pose.pose.position.z = gripper_pose['position'][2]-0.1#-0.45

				goal_pose.pose.orientation.x = gripper_pose['orientation'][0]
				goal_pose.pose.orientation.y = gripper_pose['orientation'][1]
				goal_pose.pose.orientation.z = gripper_pose['orientation'][2]
				goal_pose.pose.orientation.w = gripper_pose['orientation'][3]
				print goal_pose
				#passing the pose goal into the moveit motion planner to plan the trajectory
				left_arm.set_pose_target(goal_pose)
				left_arm.set_start_state_to_current_state()
				left_plan = left_arm.go()
				print "done planning"

				#executing planned trajectory
				rospy.sleep(5)
				# left_arm.execute(left_plan)
				# rospy.sleep(5)
				print " "
				print "*******************End pose**************************"
				print self.limb.endpoint_pose()
				left_arm.clear_pose_targets()
			else:
				print "did not see coke"

			
			
			
		# def move_to_origin(self):
		# 	lLimb = baxter.Limb('left')
		# 	rLimb = baxter.Limb('right')
		# 	fPath = '/'
		# 	pause_event = None
		# 	playPositionFile(fPath, lLimb, rLimb, pause_event)
		# 	rospy.sleep(3)

		def __init__(self):
			rospy.init_node('gotoobject')
			
			self.limb = baxter.Limb('left')
			self.object_pose = PoseStamped()
			self.confidence = 0
			self.object_index = 0
			self.coke_key="e406a3ea8a114c9ec9f2603ecc000814"
			# sub = rospy.Subscriber('/recognized_object_array', RecognizedObjectArray, self.object_callback)
			self.move_to_position(self.object_pose)
			#rospy.spin()
			


if __name__=="__main__":
	
	try:
		go = grab_object()

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