############# Baxter Speech Arm Positional Control ################

## Written by the Robotic Assistance research team
## Tufts University

## 01/2018

###################################################################
# 
# Uses a combination of user provided voice commands and pre-programmed movements
# to control the Baxter Robot from Rethink Robotics
# The movements are found in the package "taskFunctions.py"
# This program utilizes a number of packages including threading, speechRecognition, 
# and the baxter_interface package from Rethink. 

import argparse
import sys
import string
import time
from multiprocessing import Process
from threading import (
	Thread,
	activeCount,
	Event,
	enumerate,
)
import ctypes
import rospy
from std_msgs.msg import (
    UInt16,
    Bool,
)
import baxter_interface as baxter
import pyttsx
import speech_recognition as sr
from positionControl import *
from taskFunctions import *
from std_msgs.msg import String
from baxter_core_msgs.msg import CollisionDetectionState
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge

rospy.init_node('speechControl')

# Global Variables
rawCommand = ""
collisionState = False
lLimb = baxter.Limb('left')
rLimb = baxter.Limb('right')
lGripper = baxter.Gripper('left')
rGripper = baxter.Gripper('right')
# Text to speech engine
t2s = pyttsx.init()
voices = t2s.getProperty('voices')
t2s.setProperty('voice', 'english')
"""
for voice in voices:
	print(voice.id)
"""
t2s.setProperty('rate', 150)

################## Definitions #####################

# clamping function to constrain arm movement
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

# callback function when audio data is obtained
def heard(recognizer, audio):
	# Defining Commands to be accepted
	global t2s
	credsJson = ""
	with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
		credsJson = gspeechcreds.read()
	
	sens = 1
	commands = ["move", "arm", "forward", "backward", "left", "right", "up", "down", 
				"higher", "lower", "close", "hand", "open", "stop", "stop", "stop", 
                "faster", "slower", "fridge", "zero", "get", "water bottle", "fridge", 
                "place on", "table", "fridge is open", "holding something", "fridge is closed", 
                "hand is empty", "microwave", "start", "turn off", "continue", "cook",
                "put", "food", "is open", "get the food"]
	print('trying to recognize')
	try:
		commandIter = [command[0] for command in commands]
		global rawCommand
		rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', credentials_json=credsJson, preferred_phrases=commands)
		print(rawCommand)
	except sr.UnknownValueError:
		print("could not understand audio")
		pass
	except sr.RequestError as e:
		print("Recognition error; {}".format(e))

def collisionDetection(data):
	global collisionState
	global rawCommand
	collisionState = data.collision_state
	if collisionState:
		rawCommand = ""

def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    exc = ctypes.py_object(KeyboardInterrupt)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

baxter_enabler = baxter.RobotEnable(versioned=True)
baxter_enabler.enable()

# Set up subscriber for collision detection
collisionSubs = rospy.Subscriber(name='/robot/limb/left/collision_detection_state', 
								 data_class=CollisionDetectionState, callback=collisionDetection, 
								 buff_size=100)

# Start head camera display
# def image(topicData):
# 	bridge = cv_bridge.CvBridge()
# 	leftImage = bridge.imgmsg_to_cv2(topicData, desired_encoding="bgra8")
# 	cv2.imshow('Environment', leftImage)
# 	cv2.waitKey(10)

# cameraSubs = rospy.Subscriber(name='/cameras/head_camera/image', 
# 								 data_class=Image, callback=image, 
# 								 buff_size=100)

# Baxter elements 
lLimb = baxter.Limb('left')
rLimb = baxter.Limb('right')
lGripper = baxter.Gripper('left')
rGripper = baxter.Gripper('right')

# calibrating gripper
if not lGripper.calibrate():
	print("left gripper did not calibrate")
	sys.exit()

# if not rGripper.calibrate():
# 	print("right gripper did not calibrate")
# 	sys.exit()

# amp up gripper holding force
lGripper.set_holding_force(100)
lGripper.set_moving_force(100)

rGripper.set_holding_force(100)
rGripper.set_moving_force(100)

lLimb.set_joint_position_speed(.5)
lLimb.set_command_timeout(2)
strtPose = lLimb.endpoint_pose()

## This is all seting up the recognizer instance of python speech recognition
r = sr.Recognizer()
m = sr.Microphone()
r.pause_threshold = .5
r.dynamic_energy_threshold = False

#r.dynamic_energy_adjustment_damping = 0.5
#r.dynamic_energy_adjustment_ratio = 2

with m as source:
	r.adjust_for_ambient_noise(source, 1)

stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)

# Environment Tracking Booleans
env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 'microwaveOpen': False, 'holdingSomething': False, 
       'microwaveOn': False, 'foodInMicrowave': False})


slow = .07 # m/s
fast = .2 # m/s

command = ""
newCommand = ""
lastCommand = ""

# Event for pausing the current task
pause_event = Event()

#Move to Zero position
task = Thread(target=moveToDownward, args=(lLimb, rLimb, pause_event), name="movingToZero")
print(task.name)
task.daemon = True
task.start()

# Text to speech engine
t2s = pyttsx.init()
voices = t2s.getProperty('voices')
t2s.setProperty('voice', 'default')
"""
for voice in voices:
	print(voice.id)
"""
t2s.setProperty('rate', 150)

lastAliveStatus = False
lastAliveName = ""

while not rospy.is_shutdown():

	# Check if task has ended
	if lastAliveStatus and not task.is_alive():
		t2s.say("  ")
		t2s.say("  ")
		t2s.say("  ")
		t2s.say("  ")		# clear text to speech queue
		t2s.say(" Done with " + lastAliveName)
		t2s.runAndWait()

		### Set environment variables based on the task that just ended ###
		if lastAliveName == "openingFridge":
			env['fridgeOpen'] = True
		elif (lastAliveName == "gettingBottleFromStart" 
		       or lastAliveName == "gettingBottleFromOpenFridge"):
			env['fridgeOpen'] = True
			env['hasBottle'] = True
		elif lastAliveName == "closingFridge":
			env['fridgeOpen'] = False
		elif lastAliveName == "gettingBottleFromOpenFridgeAndPlacingOnTable":
			env['fridgeOpen'] = False
			env['hasBottle'] = False
			env['bottleOnTable'] = True
		elif lastAliveName == "placingOnTable":
			env['hasBottle'] = False
		elif lastAliveName == "openingMicrowave":
			env['microwaveOpen'] = True
		elif lastAliveName == "closingMicrowave":
			env['microwaveOpen'] = False
		elif lastAliveName == "turningOnMicrowave":
			env['microwaveOn'] = False
		elif lastAliveName ==  "turningOffMicrowave":
			env['microwaveOn'] = False
		elif lastAliveName == 'gettingBottlePlacingOnTable':
			env['fridgeOpen'] = False
			env['hasBottle'] = False
		elif lastAliveName == "gettingBottleFromOpenFridge":
			env['fridgeOpen'] = False
			env['hasBottle'] = False
		elif lastAliveName == "puttingFoodInMicrowave":
			env['foodInMicrowave'] = True
			env['fridgeOpen'] = False
			env['microwaveOpen'] = False
		elif lastAliveName == "gettingFoodFromMicrowave":
			env['foodInMicrowave'] = False
			env['microwaveOpen'] = False
			env['holdingSomething'] = False

	lastAliveStatus = task.is_alive()
	lastAliveName = task.name

	newCommand = rawCommand
	# Check if command has changed
	if command != newCommand: 
		speed = slow
		command = newCommand
		orient = lLimb.endpoint_pose()['orientation']
		print("last: {}, this: {}".format(lastCommand, command))

	if collisionState:
		print "Can't Move Here"
		if task and task.is_alive():
			rawCommand = "stop"
	
	##### Execute relevant command #####

	#### Directional Commands #####
	if 'move arm forward' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'y', 4, speed, pause_event), name="movingForward")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""
	if 'move arm backward' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'y', -4, speed, pause_event), name="movingBackward")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""
	if 'move arm left' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'x', -4, speed, pause_event), name="movingLeft")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""
	if 'move arm right' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'x', 4, speed, pause_event), name="movingRight")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""
	if  'arm' in command and ('higher' in command or 'up' in command):
		terminate_thread(task)
		task = Thread(target = moveOnAxis, args=(lLimb, 'z', 4, speed, pause_event), name="movingHigher")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""
	if  'arm' in command and ('lower' in command or 'down' in command):
		terminate_thread(task)
		task = Thread(target = moveOnAxis, args=(lLimb, 'z', -4, speed, pause_event), name="movingLower")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""

	### End Directional Commands ###

	### Gripper Direct Commands ###

	if 'close' in command and ('hand' in command or 'gripper' in command):
		terminate_thread(task)
		lGripper.close()
		rawCommand = ""
	if 'open' in command and ('hand' in command or 'gripper' in command):
		terminate_thread(task)
		lGripper.open()
		rawCommand = ""
	### End Gripper Direct Commands ###

	### Stop and continue Commands ###
	if 'stop' in command:
		rawCommand = ""
		pause_event.set()
	if 'continue' in command:
		rawCommand = ""
		pause_event.clear()

	### End Stop and continue commands ###

	### Begin speed commands ###
	if 'go faster' in command:
		terminate_thread(task)
		speed = fast
		command = lastCommand
		rawCommand = lastCommand
	if 'go slower' in command:
		terminate_thread(task)
		speed = slow
		command = lastCommand
		rawCommand = lastCommand

	### End speed commands ###

	### Begin task commands ###

		# Begin fridge commands # 
	if 'open' in command and 'the fridge' in command:
		terminate_thread(task)
		if not env['fridgeOpen']:
			task = Thread(target=openFridge, args=(lLimb, rLimb, pause_event), name="openingFridge")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""
	if 'get' in command and 'water bottle' in command:
		terminate_thread(task)
		if not env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=getBottleFromStart, args=(lLimb, rLimb,lGripper, pause_event), name="gettingBottleFromStart")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		if env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=pickBottleFromOpenFridge, args=(lLimb, rLimb,lGripper, pause_event), name="gettingBottleFromOpenFridge")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		if env['fridgeOpen'] and env['hasBottle']:
			pass
		rawCommand = ""
	if 'close' in command and 'the fridge' in command:
		terminate_thread(task)
		if env['fridgeOpen'] and not env['holdingSomething']:
			task = Thread(target=closeFridge, args=(lLimb, rLimb, pause_event), name="closingFridge")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""
	if 'place' in command and 'table' in command:
		terminate_thread(task)
		if env['hasBottle']:
			task = Thread(target=moveToTableAfterRetrieve, args=(lLimb, rLimb,lGripper, pause_event), name="placingOnTable")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		else:
			print("I don't have the bottle right now")
			print(task.name)
		rawCommand = ""
	if 'put' in command and 'water bottle' in command and 'table' in command:
		terminate_thread(task)
		if not env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=getBottleFull, args=(lLimb, rLimb,lGripper,pause_event), name="gettingBottlePlacingOnTable")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		elif env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=bottleOnTableAfterOpenFridge, args=(lLimb, rLimb, lGripper, pause_event), name="gettingBottleFromOpenFridgeAndPlacingOnTable")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		elif env['fridgeOpen'] and env['hasBottle']:
			task = Thread(target=moveToTableAfterRetrieve, args=(lLimb, rLimb,lGripper, pause_event), name="placingOnTable")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""

		# End Fridge task commands #

		# Water Bottle Commands #
		## Stub for now

		# Microwave commands #
	if 'open' in command and 'the microwave' in command:
		terminate_thread(task)
		if not env["microwaveOpen"] and not env["holdingSomething"]:
			task = Thread(target=openMicrowave, args=(lLimb, rLimb, pause_event), name="openingMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""
	if 'close' in command and 'microwave' in command:
		terminate_thread(task)
		if env["microwaveOpen"] and not env["holdingSomething"]:
			task = Thread(target=closeMicrowave, args=(lLimb, rLimb, pause_event), name="closingMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""
	if 'start' in command and 'microwave' in command:
		terminate_thread(task)
		if (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
			task = Thread(target=turnOnMicrowave, args=(lLimb, rLimb, pause_event), name="turningOnMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
			env["microwaveOn"] = True
		rawCommand = ""
	if ('turn off' in command) or ('stop'in command and 'microwave' in command):
		terminate_thread(task)
		if env["microwaveOn"] and (not env["holdingSomething"]):
			print('here')
			task = Thread(target=turnOffMicrowave, args=(lLimb, rLimb, pause_event), name="turningOffMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""
	if 'cook' in command and 'seconds' in command:
		terminate_thread(task)
		t = [int(s) for s in command.split() if s.isdigit()]
		print(t)
		if not t:
			print("No time given")
		elif (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
			task = Thread(target=timedMicrowave, args=(lLimb, rLimb, pause_event, t[0],), name="timedCook")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
			env["microwaveOn"] = True
		rawCommand = ""
	if (('meal' in command) or ('food' in command)) and (('put' in command or 'place' in command) and ('microwave' in command)):
		terminate_thread(task)
		if not env["fridgeOpen"] and not env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
			task = Thread(target=placeContainerInMicrowaveFromStart, args=(lLimb, rLimb, lGripper, pause_event,), name="puttingFoodInMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		if env["fridgeOpen"] and not env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
			task = Thread(target=placeContainerInMicrowaveFromOpenFridge, args=(lLimb, rLimb, lGripper, pause_event,), name="puttingFoodInMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		if not env["fridgeOpen"] and env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
			task = Thread(target=placeContainerInMicrowaveFromOpenMicrowave, args=(lLimb, rLimb, lGripper, pause_event,), name="puttingFoodInMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		if env["fridgeOpen"] and env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
			task = Thread(target=placeContainerInMicrowaveFromOpenMicOpenFridge, args=(lLimb, rLimb, lGripper, pause_event,), name="puttingFoodInMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)	
		rawCommand = ""
	if ('take' in command or 'get' in command) and ('container' in command or 'food' in command) and ('microwave' in command):
		terminate_thread(task)
		if not env["microwaveOpen"] and env["foodInMicrowave"]:
			task = Thread(target=getFoodFromMicrowave, args=(lLimb, rLimb, lGripper, pause_event,), name="gettingFoodFromMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		if env["microwaveOpen"] and env["foodInMicrowave"]:
			task = Thread(target=getFoodFromOpenMicrowave, args=(lLimb, rLimb, lGripper, pause_event,), name="gettingFoodFromMicrowave")
			pause_event.clear()
			task.daemon = True
			task.start()
			print(task.name)
		rawCommand = ""


		# end microwave Commands

	### End task related commands ###

	### environment variable control commands ###			
	if 'fridge is open' in command:
		env['fridgeOpen'] = True
	if "fridge is closed" in command:
		env['fridgeOpen'] = False
	if "holding something" in command:
		env['hasBottle'] = True
		env['holdingSomething'] = True
	if "hand is empty" in command:
		lGripper.open()
		env['hasBottle'] = False
		env['holdingSomething'] = True
	if "microwave is open" in command:
		env['microwaveOpen'] = True
	if "microwave is closed" in command:
		env['microwaveOpen'] = False
	if 'microwave is on' in command:
		env['microwaveOn'] = True
	if 'microwave is off' in command:
		env['microwaveOn'] = False 
	if 'food in microwave' in command:
		env['foodInMicrowave'] = True
	if 'microwave is empty' in command:
		env['foodInMicrowave'] = False

	### end environment variable control commands

	### miscellaneous commands ###
	if 'move to zero' in command:
		terminate_thread(task)
		task = Thread(target=moveToDownward, args=(lLimb, rLimb, pause_event), name="movingToZero")
		print(task.name)
		pause_event.clear()
		task.daemon = True
		task.start()
		rawCommand = ""
	if 'test' in command:
		terminate_thread(task)
		task = Thread(target=tester, args=(lLimb,pause_event))
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'read environment' in command:
		envString = json.dumps(env)
		print(envString)
		## Clearing text to speech backlog
		t2s.say("  ")
		t2s.say("  ")
		t2s.say("  ")
		t2s.say("  ")		
		t2s.say(envString)
		t2s.runAndWait()
		rawCommand = ""
	if 'reset environment' in command:
		env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 'microwaveOpen': False, 'holdingSomething': False, 
       'microwaveOn': False, 'foodInMicrowave': False})
		rawCommand = ""

	### End Miscellaneous commands ###

	


	newCommand = ""
	lastCommand = command
	time.sleep(.01)


# Clean Up on end
terminate_thread(task)
if env['hasBottle']:
	pause_event.clear
	rospy.init_node('ending_node')
	moveToTableAfterRetrieve(lLimb, rLimb, lGripper, pause_event)
stopListening()
print('ending')