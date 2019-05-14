import argparse
import sys
import string
import time

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface as baxter

import speech_recognition as sr

from positionControlSandbox import *

rawCommand = ""

############## Definitions #####################

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
                ("close hand", sens), ("open hand", sens), ("baxter stop", sens), 
                ("move faster", sens), ("move slower", sens)]
	print('trying to recognize')
	try:
	    global rawCommand
	    rawCommand = r.recognize_sphinx(audio_data=audio, language='en-US', keyword_entries=commands)
	    print(rawCommand)
	except sr.UnknownValueError:
	    print("could not understand audio")
	except sr.RequestError as e:
	    print("Recognition error; {0}".format(e))

rospy.init_node('speechControl')
baxter_enabler = baxter.RobotEnable(versioned=True)
baxter_enabler.enable()

# Baxter elements 
limb = baxter.Limb('left')
gripper = baxter.Gripper('left')

# calibrating gripper
if not gripper.calibrate():
	print("gripper did not calibrate")
	sys.exit()

limb.set_joint_position_speed(.8)
strtPose = limb.endpoint_pose()
strtPose['position'][0]
# Starting position of baxter
# and baxter movement parameters

curX = strtPose['position'][0]
curY = strtPose['position'][1]
curZ = strtPose['position'][2]

jointPos = xyzToAngles(curX, curY, curZ)
limb.move_to_joint_positions(jointPos)

Del = .02

r = sr.Recognizer()
m = sr.Microphone()
r.pause_threshold = .5
r.dynamic_energy_threshold = False
#r.dynamic_energy_adjustment_damping = 0.5
#r.dynamic_energy_adjustment_ratio = 2
with m as source:
	r.adjust_for_ambient_noise(source, 1)

stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)

command = ""
newCommand = ""
lastCommand = ""
while True:

	newCommand = rawCommand.split("  ")[0]
	if command != newCommand: 
		Del = .02
		command = newCommand
		
	print("last: {}, this: {}".format(lastCommand, command))

	if 'move arm forward' in command:
		curY = clamp(curY + Del, .7, 1.5)
		if xyzToAngles(curX, curY, curZ) == "invalid":
			curY = curY - Del
			print("Can't Move Here")
	if 'move arm backward' in command:
		curY = clamp(curY - Del, .7, 1.5)
		if xyzToAngles(curX, curY, curZ) == "invalid":
			curY = curY + Del
			print("Can't Move Here")
	if 'move arm left' in command:
		curX = clamp(curX - Del, -.8, .8)
		if xyzToAngles(curX, curY, curZ) == "invalid":
			curX = curX + Del
			print("Can't Move Here")
	if 'move arm right' in command:
		curX = clamp(curX + Del, -.8, .8)
		if xyzToAngles(curX, curY, curZ) == "invalid":
			curX = curX - Del
			print("Can't Move Here")
	if 'move arm higher' in command:
		curZ = clamp(curZ + Del, 0.03, .8)
		if xyzToAngles(curX, curY, curZ) == "invalid":
			curZ = curZ - Del
			print("Can't Move Here")
	if 'move arm lower' in command:
		curZ = clamp(curZ - Del, 0.03, .8)
		if xyzToAngles(curX, curY, curZ) == "invalid":
			curZ = curZ + Del
			print("Can't Move Here")
	if 'close hand' in command:
		gripper.close()
	if 'open hand' in command:
		gripper.open()
	if 'stop' in command:
		rawCommand = ""
	if 'move faster' in command:
		Del = .1
		command = lastCommand
		rawCommand = lastCommand
	if 'move slower' in command:
		Del = .02
		command = lastCommand
		rawCommand = lastCommand

	print("curX: {}, curY: {}, curZ: {}".format(curX, curY, curZ))

	jointPos = xyzToAngles(curX, curY, curZ)
	limb.move_to_joint_positions(jointPos)
	newCommand = ""
	lastCommand = command
	#time.sleep(1)