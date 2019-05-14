#! /usr/bin/env python
import rospy
import sys
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
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

	


			########################
			#					   #
			#   MASTER CALLBACK    #
			#					   #
			########################
		def master_callback(self,data):
			print data
		
					


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
		                "pick object from floor", "forward", "backward", "left", "right", "pause", "drop object", "long range", "short range"]
		    dialog = "Listening..."
		    
		    print("listening")
		    try:
		        commandIter = [command[0] for command in commands]
		        global rawCommand
		        rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', preferred_phrases=commands)
		        dialog = rawCommand
		        print("understood")
		        print(dialog)
		        self.interprete_command(dialog)
		        self.gui.label_dict['dialog'] = dialog
		        self.gui.update_labels(self.env)
		        self.gui.update_command_display()
		    except sr.UnknownValueError:
		        dialog = "Listening..."
		        pass
		    except sr.RequestError as e:
		        print("Recognition error; {}".format(e))


		def interprete_command(self, command):
			if 'localized' in command:
				self.env['robotlocalized'] = True

			if 'open the fridge' in command:
				self.env['fridgeOpen'] = True

			if 'close the fridge' in command:
				self.env['fridgeOpen'] = False

			if 'open the microwave' in command:
				self.env['microwaveOpen'] = True

			if 'close the microwave' in command:
				self.env['microwaveOpen'] = False

			if 'get a water bottle' in command:
				self.env['bottleInFridge'] = False
				self.env['hasBottle'] = True

			if 'put on table' in command:
				self.env['bottleOnTable'] = True
				self.env['hasBottle'] = False

			if 'put food in microwave' in command:
				self.env['foodInFridge'] = False
				self.env['foodInMicrowave'] = True

			if 'get food from microwave' in command:
				self.env['foodInFridge'] = False
				self.env['foodInMicrowave'] = False


		def __init__(self):
			rospy.init_node('vision_move', disable_signals=True)
			
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

			self.worked = False

			#Environment variables
			self.env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 
				'bottleInFridge': True, 'microwaveOpen': False, 'holdingSomething': False, 
				'microwaveOn': False, 'foodInMicrowave': False, 'foodInFridge': True, 
				'foodOnTable': False, 'robotlocalized': False, 'mobileBaseActivated': False})

			#Origin joint angles
			self.origin = {'left_w0': -0.07286408742455715, 
			'left_w1': 1.180398216277826, 
			'left_w2': -1.7725148004015956, 
			'left_e0': 0.17257283863710904, 
			'left_e1': 0.05138835639416136, 
			'left_s0': 0.6879903833666081, 
			'left_s1': -0.5414952181235511
			}
			
			#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			rospy.Subscriber('/cmd_vel', Twist, self.master_callback)
			

			self.setup_speech()
			self.stopListening = self.rec.listen_in_background(self.mic, self.speech_callback, phrase_time_limit=4)
			
			#update command display in gui
			
			
			self.gui.main_loop()
			#print self.lLimb.endpoint_pose()
			#print self.lLimb.joint_angles()
			
			
			rospy.spin()
			
			


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass

