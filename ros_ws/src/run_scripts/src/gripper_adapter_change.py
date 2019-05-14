#! /usr/bin/env python
import rospy
import sys
import baxter_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from positionControl import *
from mobileTasks import *
import time
import tf2_ros
import tf2_geometry_msgs
import tf
import speech_recognition as sr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class pick_object:

	def setup_speech(self):
		    print "in speech setup"
		    self.rec.pause_threshold = .5
		    self.rec.dynamic_energy_threshold = False
		    with self.mic as source:
		        self.rec.adjust_for_ambient_noise(source, 3)


	def speech_callback(self, recognizer, audio):
	    # Defining Commands to be accepted
	    global t2s, dialog
	    
	    sens = 1
	    commands = ["pick up object", "move to origin"]
	    dialog = "Listening..."
	    print("listening")
	    ang = self.lLimb.joint_angles()
	    print ('wrist angle: ',ang['left_w2'] )
		
	    try:
	        #commandIter = [command[0] for command in commands]
	        global rawCommand
	        rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', preferred_phrases=commands)
	        dialog = rawCommand
	        print("understood")
	        print(dialog)
	        self.interprete_command(rawCommand)

	    except sr.UnknownValueError:
	        dialog = "Listening..."
	        pass
	    except sr.RequestError as e:
	        print("Recognition error; {}".format(e))


	
	def interprete_command(self,command):
		if 'pick' in command or 'object' in command:
			playPositionFile('waypoints/origin.wp', self.lLimb, self.rLimb, self.pause_event)
			self.grab_object()

		elif 'move' in command or 'origin' in command:
			playPositionFile('waypoints/origin.wp', self.lLimb, self.rLimb, self.pause_event)
			


	def orientation_callback(self,angle):
		self.object_orientation = angle.data 


	def bottle_callback(self, pose):
		self.bottle_pose = pose
		# print(self.transform_bottle_pose_to_robot_rf())

		#self.grab_object()

	def cup_callback(self, pose):
		self.cup_pose = pose
		# print(self.transform_cup_pose_to_robot_rf())
		#self.grab_object()


	def transform_bottle_pose_to_robot_rf(self):
		#kinect camera axis not the same as the robot axis so we could have
		#to perform the necessary transforms first to get both axes aligned
		#and then to transform camera rf to robot's rf
		#goal_pose is the final pose of the marker wrt the robot's rf

		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
			rospy.Duration(1.0))
		trans_pose = tf2_geometry_msgs.do_transform_pose(self.bottle_pose, transform)

		return trans_pose


	def transform_cup_pose_to_robot_rf(self):
		#kinect camera axis not the same as the robot axis so we could have
		#to perform the necessary transforms first to get both axes aligned
		#and then to transform camera rf to robot's rf
		#goal_pose is the final pose of the marker wrt the robot's rf

		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
		tf_listener = tf2_ros.TransformListener(tf_buffer)

		transform = tf_buffer.lookup_transform('base', 'camera_link',rospy.Time(0),
			rospy.Duration(1.0))
		trans_pose = tf2_geometry_msgs.do_transform_pose(self.cup_pose, transform)

		return trans_pose



	def grab_bottle(self):
		pose = self.transform_bottle_pose_to_robot_rf()
		# print(pose)
		gpose = get_pick_bottle_drink_pose(pose)
		# self.engage_gripper()
		time.sleep(2)
		playPositionFile('waypoints/get_ready_to_grab.wp', self.lLimb, self.rLimb, self.pause_event)
		move_to_goal_pose(self.lLimb, gpose, self.pause_event)
		# time.sleep(2)
		self.lGripper.close()
		# playPositionFile('waypoints/transport_object.wp', self.lLimb, self.rLimb, self.pause_event)
		# self.lGripper.open()
		# playPositionFile('waypoints/leave_to_origin.wp', self.lLimb, self.rLimb, self.pause_event)
		# time.sleep(2)
		# playPositionFile('waypoints/remove_gripper.wp', self.lLimb, self.rLimb, self.pause_event)

	def pour_drink(self):
		playPositionFile('waypoints/raise_before_pour.wp', self.lLimb, self.rLimb, self.pause_event)
		pose = self.transform_cup_pose_to_robot_rf()
		p = get_pour_drink_pose(pose)
		move_to_goal_pose(self.lLimb, p, self.pause_event)
		time.sleep(3)
		playPositionFile('waypoints/return_bottle_to_table.wp', self.lLimb, self.rLimb, self.pause_event)
		time.sleep(1)
		self.lGripper.open()
		playPositionFile('waypoints/retract_arm_from_bottle.wp', self.lLimb, self.rLimb, self.pause_event)
		

	def engage_gripper(self):
		# print('width is',self.object_width)
		playPositionFile('waypoints/wear_gripper.wp', self.lLimb, self.rLimb, self.pause_event)
		
		# if self.object_width > 180.0:
		# 	if self.adapter_on is False:
		# 		playPositionFile('waypoints/wear_gripper.wp', self.lLimb, self.rLimb, self.pause_event)
		# 		self.adapter_on = True
		# else:
		# 	if self.adapter_on is True:
		# 		playPositionFile('waypoints/remove_gripper.wp', self.lLimb, self.rLimb, self.pause_event)
		# 		self.adapter_on = False

	
	def remove_gripper(self):
		playPositionFile('waypoints/remove_gripper.wp', self.lLimb, self.rLimb, self.pause_event)


	def move_cup(self):
		pose = self.transform_cup_pose_to_robot_rf()
		p = get_move_cup_pose(pose)
		playRightPositionFile('waypoints/pick_the_cup.wp', self.lLimb, self.rLimb, self.pause_event)
		time.sleep(1)
		right_move_to_goal_pose(self.rLimb, p, self.pause_event)
		self.rGripper.close()



	def map_angle_to_wrist(self):
		x = self.object_orientation

		if (0 <= x <= 15) or (165 <=x <= 180):
			return -0.31

		
		elif 15 < x < 90:
			in_min = 10; in_max = 90; out_min = -0.31; out_max = 1.195;

		elif 90 <= x < 165:
			in_min = 90; in_max = 170; out_min = -0.31; out_max = -2.02;

		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


	def __init__(self):
		rospy.init_node('pick_object', disable_signals=True)
		self.limb = baxter.Limb('left')
		self.place = 0
		self.baxter_enabler = baxter.RobotEnable(versioned=True)
		self.baxter_enabler.enable()

		self.lLimb = baxter.Limb('left')
		self.rLimb = baxter.Limb('right')
		self.lGripper = baxter.Gripper('left')
		self.rGripper = baxter.Gripper('right')
		# print self.lLimb.endpoint_pose()

		#calibrating gripper
		if not self.lGripper.calibrate():
		    print("left gripper did not calibrate")
		    sys.exit()
		if not self.rGripper.calibrate():
			print('right gripper did not calibrate')
			sys.exit()

		self.lGripper.set_holding_force(100)
		self.lGripper.set_moving_force(100)

		self.rGripper.set_holding_force(100)
		self.rGripper.set_moving_force(100)
		# self.lGripper.open()
		self.pause_event = Event()
		#ang = self.lLimb.joint_angles()
		# print self.lLimb.endpoint_pose()
		#ang['left_w2'] = -0.75
		#move_to_goal_joint_angle(self.lLimb, ang, self.pause_event)

		

		self.head = baxter.Head()
		self.adapter_on = False
		#self.head.set_pan(1.57)

		#Speech recognition variables
		# self.rec = sr.Recognizer()
		# self.mic = sr.Microphone()

		self.object_orientation = None
		self.object_pose = None
		self.pause_event = Event()

		#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
		rospy.Subscriber('/bottle_pose', PoseStamped, self.bottle_callback)
		rospy.Subscriber('/cup_pose', PoseStamped, self.cup_callback)
		
		time.sleep(3)

		# self.setup_speech()
		# self.stopListening = self.rec.listen_in_background(self.mic, self.speech_callback, phrase_time_limit=4)
		
		self.engage_gripper()
		self.grab_bottle()
		time.sleep(1)
		self.move_cup()
		self.pour_drink()
		time.sleep(1)
		self.remove_gripper()
		
		

		# print self.rLimb.endpoint_pose()

		# print self.lLimb.endpoint_pose()
		#print self.lLimb.joint_angles()
		# self.grab_bottle()
		
		rospy.spin()
		

if __name__=="__main__":
	
	try:
		go = pick_object()

	except rospy.ROSInterruptException: pass