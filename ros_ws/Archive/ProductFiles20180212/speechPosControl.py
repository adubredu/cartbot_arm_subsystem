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
from threading import Thread
import ctypes

import rospy

from std_msgs.msg import (
    UInt16,
    Bool,
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
                ('open the fridge', sens), ('move to zero', sens), ('test', sens)]
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

def placeHolder(lLimb):
	print(lLimb.endpoint_pose())

def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    exc = ctypes.py_object(SystemExit)
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

r = sr.Recognizer()
m = sr.Microphone()
r.pause_threshold = .5
r.dynamic_energy_threshold = False
#r.dynamic_energy_adjustment_damping = 0.5
#r.dynamic_energy_adjustment_ratio = 2
with m as source:
	r.adjust_for_ambient_noise(source, 1)

stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)

speed = .03

command = ""
newCommand = ""
lastCommand = ""
orient = baxter.Limb.Quaternion(0, 0, 0, 0)
task = Thread(target=placeHolder, args=(lLimb,))
task.start()

while not rospy.is_shutdown():

	newCommand = rawCommand.split("  ")[0]
	if command != newCommand: 
		speed = .05
		command = newCommand
		orient = lLimb.endpoint_pose()['orientation']
		print("last: {}, this: {}".format(lastCommand, command))

	if collisionState:
		print "Can't Move Here"
		if task and task.is_alive():
			terminate_thread(task)
		
	if 'move arm forward' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'y', 4, speed,))
		task.start()
		rawCommand = ""
	if 'move arm backward' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'y', -4, speed,))
		task.start()
		rawCommand = ""
	if 'move arm left' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'x', -4, speed,))
		task.start()
		rawCommand = ""
	if 'move arm right' in command:
		terminate_thread(task)
		task = Thread(target=moveOnAxis, args=(lLimb, 'x', 4, speed,))
		task.start()
		rawCommand = ""
	if 'move arm higher' in command:
		terminate_thread(task)
		task = Thread(target = moveOnAxis, args=(lLimb, 'z', 4, speed,))
		task.start()
		rawCommand = ""
	if 'move arm lower' in command:
		terminate_thread(task)
		task = Thread(target = moveOnAxis, args=(lLimb, 'z', -4, speed,))
		task.start()
		rawCommand = ""
	if 'close hand' in command:
		lGripper.close()
	if 'open hand' in command:
		lGripper.open()
	if 'stop' in command:
		rawCommand = ""
		terminate_thread(task)
	if 'go faster' in command:
		terminate_thread(task)
		speed = .08
		command = lastCommand
		rawCommand = lastCommand
	if 'go slower' in command:
		terminate_thread(task)
		speed = .03
		command = lastCommand
		rawCommand = lastCommand
	if 'open the fridge' in command:
		terminate_thread(task)
		task = Thread(target=openFridge, args=(lLimb, rLimb,))
		task.start()
		rawCommand = ""
	if 'move to zero' in command:
		terminate_thread(task)
		task = Thread(target=moveLeftToDatum, args=(lLimb, rLimb,))
		task.start()
		rawCommand = ""
	if 'test' in command:
		terminate_thread(task)
		task = Thread(target=tester, args=(lLimb,))
		task.start()
		rawCommand = ""

	newCommand = ""
	lastCommand = command
	if task.is_alive():
		print(task.name)

print('ending')