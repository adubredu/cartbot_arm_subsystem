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
				#self.start_listen()
				self.place = 4
				if self.place != 0:
					self.move_to_position(data)

				else:
					self.move_to_origin()
		


		def move_to_position(self, data):
			moveit_commander.roscpp_initialize(sys.argv)
			# from_frame = "/torso"
			# to_frame = '/ar_marker_5'
			listener = tf.TransformListener()
			# if not listener.frameExists(from_frame):# or not listener.frameExists(to_frame):
			# 	print 'Frames not found'
		 #    	exit(0)
			# found_object = False
			robot = moveit_commander.RobotCommander()
			scene = moveit_commander.PlanningSceneInterface()
			left_arm = moveit_commander.MoveGroupCommander('left_arm')
			left_arm.set_planner_id('RRTConnectkConfigDefault')
			left_arm.set_planning_time(10)
		
			for i in range(0,len(data.markers)):
				if data.markers[i].id == self.place:
					found_object = True
					marker_pose = data.markers[i].pose
					# current_pose = self.limb.endpoint_pose()
					
					

					# print marker_pose
					print marker_pose
					print "----------------------------"
					print " "
					# t = listener.getLatestCommonTime(from_frame, to_frame)
    	# 			position, quaternion = listener.lookupTransform(from_frame, to_frame, t)

					# broadcaster = tf2_ros.StaticTransformBroadcaster()
					# static_transformStamped = geometry_msgs.msg.TransformStamped()
					
					# static_transformStamped.header.stamp = rospy.Time.now()
					# static_transformStamped.header.frame_id = "head_camera"
					# static_transformStamped.child_frame_id = 'camera_link'
					
					# static_transformStamped.transform.translation.x = 0
					# static_transformStamped.transform.translation.y = 0
					# static_transformStamped.transform.translation.z = 0
					
					# quat = tf.transformations.quaternion_from_euler(0,-1.57,-1.57)
					# static_transformStamped.transform.rotation.x = quat[0]
					# static_transformStamped.transform.rotation.y = quat[1]
					# static_transformStamped.transform.rotation.z = quat[2]
					# static_transformStamped.transform.rotation.w = quat[3]

					# broadcaster.sendTransform(static_transformStamped)
					# rospy.sleep(3.0)
					print "gripper position"
					print "******************************"
					limb = baxter_interface.Limb('left')
					gripper_pose = limb.endpoint_pose()
					print gripper_pose

					tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
					tf_listener = tf2_ros.TransformListener(tf_buffer)

					# listener.waitForTransform('base', 'camera_link', rospy.Time.now(), rospy.Duration(5.0))
					transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
						rospy.Duration(1.0))
					goal_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, transform)
					print goal_pose

					# goal_pose.pose.position.z -= 0.23
					goal_pose.pose.position.y -= 0.03

					goal_pose.pose.orientation.x = gripper_pose['orientation'][0]
					goal_pose.pose.orientation.y = gripper_pose['orientation'][1]
					goal_pose.pose.orientation.z = gripper_pose['orientation'][2]
					goal_pose.pose.orientation.w = gripper_pose['orientation'][3]
					# broadcaster = tf2_ros.StaticTransformBroadcaster()
					# static_transformStamped = geometry_msgs.msg.TransformStamped()
					
					# static_transformStamped.header.stamp = rospy.Time.now()
					# static_transformStamped.header.frame_id = "base"
					# static_transformStamped.child_frame_id = 'test'
					
					# static_transformStamped.transform.translation.x = goal_pose.pose.position.x
					# static_transformStamped.transform.translation.y = goal_pose.pose.position.y
					# static_transformStamped.transform.translation.z = goal_pose.pose.position.z
					
					
					# static_transformStamped.transform.rotation.x = goal_pose.pose.orientation.x
					# static_transformStamped.transform.rotation.y = goal_pose.pose.orientation.y
					# static_transformStamped.transform.rotation.z = goal_pose.pose.orientation.z
					# static_transformStamped.transform.rotation.w = goal_pose.pose.orientation.w

					# broadcaster.sendTransform(static_transformStamped)

					left_arm.set_pose_target(goal_pose)
					left_arm.set_start_state_to_current_state()

					left_plan = left_arm.plan()
					print "done planning"
					rospy.sleep(5)
					# raw_input('Press <Enter> to move: ')
					left_arm.execute(left_plan)
    				# goal_pose = PoseStamped()
    				# goal_pose.header.frame_id = "it"
    				# goal_pose.pose.position.x = position[0]
    				# goal_pose.pose.position.y = position[1]
    				# goal_pose.pose.position.z = position[2] + 0.1
    		# 		print goal_pose
    		# 		goal_joint_angles = xyzToAngles('left', goal_pose.pose.position.x, 
						# goal_pose.pose.position.y, goal_pose.pose.position.z-0.1,
						# current_pose['orientation'][0], current_pose['orientation'][1], 
						# current_pose['orientation'][2], current_pose['orientation'][3])

    		# 		if goal_joint_angles != 'invalid':
						# current_joint_angles = self.limb.joint_angles()
						# wpArray = []
						# wpArray.append(json.loads(json.dumps(current_joint_angles)))
						# wpArray.append(json.loads(json.dumps(goal_joint_angles)))

						# path, period = splineWaypoints(wpArray)

						# self.limb.set_joint_position_speed(.7)

						# rate = rospy.Rate(1/(period))

						# for i in range(len(path)):
						# 	if path[i] != 'invalid':
						# 		self.limb.set_joint_positions(path[i])
						# 		rate.sleep()


			# if not found_object:
			# 	print "Could not find it"





		def move_to_origin(self):
			current_joint_angles = self.limb.joint_angles()
			wpArray = []
			wpArray.append(json.loads(json.dumps(current_joint_angles)))
			wpArray.append(json.loads(json.dumps(self.origin_joint_angles)))

			path, period = splineWaypoints(wpArray)

			self.limb.set_joint_position_speed(.7)

			rate = rospy.Rate(1/period)

			for i in range(len(path)):
				if path[i] != 'invalid':
					self.limb.set_joint_positions(path[i])
					rate.sleep()


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


		def __init__(self):
			rospy.init_node('gotomarker')
			print "What can I do for you?"
			self.limb = baxter.Limb('left')
			self.place = 0
			self.origin_joint_angles = self.limb.joint_angles()

			sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback)
			rospy.spin()


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass