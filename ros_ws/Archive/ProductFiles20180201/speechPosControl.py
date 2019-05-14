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
import multiprocessing

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface as baxter

import speech_recognition as sr

import positionControlPackage

from positionControlPackage import *
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

################## Definitions #####################

# clamping function to constrain arm movement
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

# callback function when audio data is obtained
def heard(recognizer, audio):
	# Defining Commands to be accepted
	sens = 1
	commands = [("baxter go forward", sens), ("baxter go backward", sens), 
                ("move arm forward", sens), ("move arm backward", sens), ("move arm left", sens), 
                ("move arm right", sens), ("move arm higher", sens), ("move arm lower", sens), 
                ("close hand", sens), ("open hand", sens), ("stop", sens), 
                ("go faster", sens), ("go slower", sens), ("right arm", sens), 
                ('open the fridge', sens), ('move to zero', sens)]
	print('trying to recognize')
	try:
	    global rawCommand
	    rawCommand = r.recognize_sphinx(audio_data=audio, language='en-US', keyword_entries=commands)
	    print(rawCommand)
	except sr.UnknownValueError:
	    print("could not understand audio")
	except sr.RequestError as e:
	    print("Recognition error; {0}".format(e))

def collisionDetection(data):
	global collisionState
	global rawCommand
	collisionState = data.collision_state
	if collisionState:
		rawCommand = ""



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

r = sr.Recognizer()
m = sr.Microphone()
r.pause_threshold = .5
r.dynamic_energy_threshold = False
#r.dynamic_energy_adjustment_damping = 0.5
#r.dynamic_energy_adjustment_ratio = 2
with m as source:
	r.adjust_for_ambient_noise(source, 1)

stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)

speed = .05

command = ""
newCommand = ""
lastCommand = ""
while True:

	newCommand = rawCommand.split("  ")[0]
	if command != newCommand: 
		speed = .05
		command = newCommand
		orient = lLimb.endpoint_pose()['orientation']

	if collisionState:
		print "Can't Move Here"
		command = ""
		rawCommand = ""
		
	print("last: {}, this: {}".format(lastCommand, command))

	if 'move arm forward' in command:
		moveOnAxis(lLimb, 'y', .01, speed)
	if 'move arm backward' in command:
		moveOnAxis(lLimb, 'y', -.01, speed)
	if 'move arm left' in command:
		moveOnAxis(lLimb, 'x', -.01, speed)
	if 'move arm right' in command:
		moveOnAxis(lLimb, 'x', .01, speed)
	if 'move arm higher' in command:
		moveOnAxis(lLimb, 'z', .01, speed)
	if 'move arm lower' in command:
		moveOnAxis(lLimb, 'z', -.01, speed)
	if 'close hand' in command:
		lGripper.close()
	if 'open hand' in command:
		lGripper.open()
	if 'stop' in command:
		rawCommand = ""
	if 'go faster' in command:
		speed = .1
		command = lastCommand
		rawCommand = lastCommand
	if 'go slower' in command:
		speed = .05
		command = lastCommand
		rawCommand = lastCommand
	if 'open the fridge' in command:
		openFridge(lLimb, rLimb)
		rawCommand = ""
	if 'move to zero' in command:
		moveLeftToDatum(lLimb, rLimb)
		rawCommand = ""

	newCommand = ""
	lastCommand = command