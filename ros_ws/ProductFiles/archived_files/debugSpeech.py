#! /usr/bin/env python

import speech_recognition as sr

rawCommand=""

# def heard(recognizer, audio):
# 	# Defining Commands to be accepted
# 	global t2s
# 	credsJson = ""
# 	with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
# 		credsJson = gspeechcreds.read()
	
# 	sens = 1
# 	commands = ["move", "arm", "forward", "backward", "left", "right", "up", "down", 
# 				"higher", "lower", "close", "hand", "open", "stop", "stop", "stop", 
#                 "faster", "slower", "fridge", "zero", "get", "water bottle", "fridge", 
#                 "place on", "table", "fridge is open", "holding something", "fridge is closed", 
#                 "hand is empty", "microwave", "start", "turn off", "continue", "cook",
#                 "put", "food", "is open", "get the food"]
# 	print('trying to recognize')
# 	try:
# 		commandIter = [command[0] for command in commands]
# 		global rawCommand
# 		rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', credentials_json=credsJson, preferred_phrases=commands)
# 		print(rawCommand)
# 	except sr.UnknownValueError:
# 		print("could not understand audio")
# 		pass
# 	except sr.RequestError as e:
# 		print("Recognition error; {}".format(e))


r = sr.Recognizer()
m = sr.Microphone()
r.pause_threshold = .5
r.dynamic_energy_threshold = False


def speech_callback(recognizer, audio):
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
		        
		    except sr.UnknownValueError:
		        dialog = "Listening..."
		        pass
		    except sr.RequestError as e:
		        print("Recognition error; {}".format(e))


with m as source:
	r.adjust_for_ambient_noise(source, duration=5)
	print("speak")
	audio = r.listen(source)

# with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
#  		credsJson = gspeechcreds.read()
# credsJson = GOOGLE_APPLICATION_CREDENTIALS
'''
print("listening")
commands = ["move arm forward", "backward", "left", "right", "up", "down", 
				"higher", "lower", "close", "hand", "open", "stop", "stop", "stop", 
                "faster", "slower", "open the fridge", "zero", "get", "water bottle", "fridge", 
                "place on", "table", "fridge is open", "holding something", "fridge is closed", 
                "hand is empty", "put food in the microwave", "start", "turn off", "continue", "cook",
                "put", "food", "is open", "get the food"]
print('trying to recognize')
try:
	# commandIter = [command[0] for command in commands]
	rawCommand = r.recognize_google_cloud(audio_data=audio, language='en-US', preferred_phrases=commands)
	print(rawCommand)
except sr.UnknownValueError:
	print("could not understand audio")
	pass
except sr.RequestError as e:
	print("Recognition error; {}".format(e))

'''
# while True:
# 	with m as source:
# 		r.adjust_for_ambient_noise(source, 1)
# 	stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)
# 	print rawCommand
stopListening = r.listen_in_background(m, speech_callback, phrase_time_limit=4)
while True:
	pass
