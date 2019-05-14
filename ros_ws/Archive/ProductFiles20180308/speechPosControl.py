############# Baxter Speech Arm Positional Control ################

## Written by the Robotic Assistance research team
## Tufts University

## 01/2018

###################################################################
# 
# Controls the X, Y, Z position of the end effector of one arm of a baxter robot
# Uses CMU Sphinx to decode audio data into usable commands
# Uses speech_recognition python package to record audio data

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

import Queue as Q

from positionControl import *
from taskFunctions import *

from std_msgs.msg import String

from baxter_core_msgs.msg import CollisionDetectionState

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
	with open('baxter-helper-bot-a4bcdf369576.json', 'r') as gspeechcreds:
		credsJson = gspeechcreds.read()
	
	sens = 1
	commands = [("baxter go forward", sens), ("baxter go backward", sens), 
                ("move arm forward", sens), ("move arm backward", sens), ("move arm left", sens), 
                ("move arm right", sens), ("move arm higher", sens), ("move arm lower", sens), 
                ("close hand", sens), ("open hand", sens), ("stop", sens), 
                ("go faster", sens), ("go slower", sens), ("right arm", sens), 
                ('open the fridge', sens), ('move to zero', sens), 
                ('get a water bottle', sens), ("close the fridge", sens), ("place on the table", sens), 
                ("fridge is open", sens), ("holding something", sens), ("fridge is closed", sens), 
                ("hand is empty", sens), ("open the microwave", sens), ("close the microwave", sens), 
                ("start the microwave", sens), ("turn off", sens), ("stop the microwave", sens),
                ("put a water bottle on the table", sens), ("continue", sens)]
	print('trying to recognize')
	try:
		commandIter = [command[0] for command in commands]
		global rawCommand
		rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', credentials_json=credsJson, preferred_phrases=commandIter)
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

# Baxter elements 
lLimb = baxter.Limb('left')
rLimb = baxter.Limb('right')
lGripper = baxter.Gripper('left')
rGripper = baxter.Gripper('right')

# calibrating gripper
if not lGripper.calibrate():
	print("left gripper did not calibrate")
	sys.exit()

if not rGripper.calibrate():
	print("right gripper did not calibrate")
	sys.exit()

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
env = ({'fridgeOpen': False, 'hasBottle': False, 'microwaveOpen': False, 'holdingSomething': False, 
       'microwaveOn': False})


slow = .07
fast = .2

command = ""
newCommand = ""
lastCommand = ""

# Event for pausing the current task
pause_event = Event()

#Move to Zero position
task = Thread(target=moveToDownward, args=(lLimb, rLimb, pause_event), name="movingToZero")
print(task.name)
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

	if lastAliveStatus and not task.is_alive():
		t2s.say("  ")
		t2s.say("  ")
		t2s.say("  ")
		t2s.say("  ")		
		t2s.say(" Done with " + lastAliveName)
		t2s.runAndWait()
		if lastAliveName == "openingFridge":
			env['fridgeOpen'] = True
		elif (lastAliveName == "gettingBottleFromStart" 
		       or lastAliveName == "gettingBottleFromOpenFridge"):
			env['fridgeOpen'] = True
			env['hasBottle'] = True
		elif lastAliveName == "closingFridge":
			env['fridgeOpen'] = False
		elif lastAliveName == "placingOnTable":
			env['hasBottle'] = False
		elif lastAliveName == "openingMicrowave":
			env['microwaveOpen'] = True
		elif lastAliveName == "closingMicrowave":
			env['microwaveOpen'] = False
		elif lastAliveName == "turningOnMicrowave":
			env['microwaveOn'] = True
		elif lastAliveName ==  "turningOffMicrowave":
			env['microwaveOn'] = False
		elif lastAliveName == 'gettingBottlePlacingOnTable':
			env['fridgeOpen'] = False
			env['hasBottle'] = False
		elif lastAliveName == "gettingBottleFromOpenFridge":
			env['fridgeOpen'] = False
			env['hasBottle'] = False

	lastAliveStatus = task.is_alive()
	lastAliveName = task.name

	newCommand = rawCommand.split("  ")[0]
	if command != newCommand: 
		speed = slow
		command = newCommand
		orient = lLimb.endpoint_pose()['orientation']
		print("last: {}, this: {}".format(lastCommand, command))

	if collisionState:
		print "Can't Move Here"
		if task and task.is_alive():
			terminate_thread(task)
	
	##### Execute relevant command #####
	if 'move arm forward' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'y', 4, speed, pause_event), name="movingForward")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'move arm backward' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'y', -4, speed, pause_event), name="movingBackward")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'move arm left' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'x', -4, speed, pause_event), name="movingLeft")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'move arm right' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'x', 4, speed, pause_event), name="movingRight")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'move arm higher' in command:
		terminate_thread(task)
		task = Thread(target = moveOnAxis, args=(lLimb, 'z', 4, speed, pause_event), name="movingHigher")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'move arm lower' in command:
		terminate_thread(task)
		task = Thread(target = moveOnAxis, args=(lLimb, 'z', -4, speed, pause_event), name="movingLower")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if ' close ' and (' hand ' or ' gripper ') in command:
		lGripper.close()
	if ' open ' and (' hand ' or ' gripper ') in command:
		lGripper.open()
	if 'stop' in command:
		rawCommand = ""
		pause_event.set()
	if 'continue' in command:
		rawCommand = ""
		pause_event.clear()
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
	if ' open ' and ' fridge ' in command:
		terminate_thread(task)
		if not env['fridgeOpen']:
			task = Thread(target=openFridge, args=(lLimb, rLimb, pause_event), name="openingFridge")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if ' get ' and ' water bottle ' in command:
		terminate_thread(task)
		if not env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=getBottleFromStart, args=(lLimb, rLimb,lGripper, pause_event), name="gettingBottleFromStart")
			pause_event.clear()
			task.start()
			print(task.name)
		if env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=pickBottleFromOpenFridge, args=(lLimb, rLimb,lGripper, pause_event), name="gettingBottleFromOpenFridge")
			pause_event.clear()
			task.start()
			print(task.name)
		if env['fridgeOpen'] and env['hasBottle']:
			pass
		rawCommand = ""
	if 'close the fridge' in command:
		terminate_thread(task)
		if env['fridgeOpen']:
			task = Thread(target=closeFridge, args=(lLimb, rLimb, pause_event), name="closingFridge")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if 'place on the table' in command:
		terminate_thread(task)
		if env['hasBottle']:
			task = Thread(target=moveToTableAfterRetrieve, args=(lLimb, rLimb,lGripper, pause_event), name="placingOnTable")
			pause_event.clear()
			task.start()
			print(task.name)
		else:
			print("I don't have the bottle right now")
			print(task.name)
		rawCommand = ""
	if "put a water bottle on the table" in command:
		terminate_thread(task)
		if not env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=getBottleFull, args=(lLimb, rLimb,lGripper,pause_event), name="gettingBottlePlacingOnTable")
			pause_event.clear()
			task.start()
			print(task.name)
		elif env['fridgeOpen'] and not env['hasBottle']:
			task = Thread(target=bottleOnTableAfterOpenFridge, args=(lLimb, rLimb, lGripper, pause_event), name="gettingBottleFromOpenFridge")
			pause_event.clear()
			task.start()
			print(task.name)
		elif env['fridgeOpen'] and env['hasBottle']:
			task = Thread(target=moveToTableAfterRetrieve, args=(lLimb, rLimb,lGripper, pause_event), name="placingOnTable")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if 'open the microwave' in command:
		terminate_thread(task)
		if not env["microwaveOpen"] and not env["holdingSomething"]:
			task = Thread(target=openTheMicrowave, args=(lLimb, rLimb, pause_event), name="openingMicrowave")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if 'close the microwave' in command:
		terminate_thread(task)
		if env["microwaveOpen"] and not env["holdingSomething"]:
			task = Thread(target=closeTheMicrowave, args=(lLimb, rLimb, pause_event), name="closingMicrowave")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if 'start the microwave' in command:
		terminate_thread(task)
		if (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
			task = Thread(target=turnOnMicrowave, args=(lLimb, rLimb, pause_event), name="turningOnMicrowave")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if 'turn off' in command or 'stop the microwave' in command:
		terminate_thread(task)
		print(env)
		if env["microwaveOn"] and (not env["holdingSomething"]):
			print('here')
			task = Thread(target=turnOffMicrowave, args=(lLimb, rLimb, pause_event), name="turningOffMicrowave")
			pause_event.clear()
			task.start()
			print(task.name)
		rawCommand = ""
	if 'fridge is open' in command:
		env['fridgeOpen'] = True
	if "fridge is closed" in command:
		env['fridgeOpen'] = False
	if "holding something" in command:
		env['hasBottle'] = True
	if "hand is empty" in command:
		lGripper.open()
		env['hasBottle'] = False
	if 'move to zero' in command:
		terminate_thread(task)
		task = Thread(target=moveToDownward, args=(lLimb, rLimb, pause_event), name="movingToZero")
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""
	if 'test' in command:
		terminate_thread(task)
		task = Thread(target=tester, args=(lLimb,pause_event))
		print(task.name)
		pause_event.clear()
		task.start()
		rawCommand = ""

	newCommand = ""
	lastCommand = command
	time.sleep(.01)

terminate_thread(task)
if env['hasBottle']:
	pause_event.clear
	rospy.init_node('ending_node')
	moveToTableAfterRetrieve(lLimb, rLimb, lGripper, pause_event)
stopListening()
print('ending')