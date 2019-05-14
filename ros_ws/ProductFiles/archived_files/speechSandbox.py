#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

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

## This is all seting up the recognizer instance of python speech recognition
r = sr.Recognizer()
m = sr.Microphone()#device_index=0)
r.pause_threshold = .5
r.dynamic_energy_threshold = False

with m as source:
	print("please wait and keep quiet, adjusting for ambient noise")
	r.adjust_for_ambient_noise(source, 3)
	print("ok, adjusted!")

stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)

global rawCommand
rawCommand = ""

while True:
	time.sleep(1)