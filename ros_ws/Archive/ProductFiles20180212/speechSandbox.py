#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

import speech_recognition as sr

# obtain audio from the microphone
sens = 1

commands = [("baxter go forward", sens), ("baxter go backward", sens), 
            ("move arm forward", sens), ("move arm backward", sens), ("move arm left", sens), 
            ("move arm right", sens), ("move arm higher", sens), ("move arm lower", sens), 
            ("close hand", sens), ("open hand", sens), ("baxter stop", sens)]

credsJson = ""
with open('gspeechcreds.json', 'r') as gspeechcreds:
    credsJson = gspeechcreds.read()

r = sr.Recognizer()
command = ""
while True:
	commandSaid = False
	with sr.Microphone(device_index=4) as source:
	    print("Say something")
	    try:
	        audio = r.listen(source, timeout=1)
	        commandSaid = True
	    except sr.WaitTimeoutError:
	        commandSaid = False
	        pass

	if commandSaid:
	# recognize speech using Sphinx
		print('trying to recognize')
		try:
		    command = r.recognize_sphinx(audio_data=audio, language='en-US', keyword_entries=commands)
		except sr.UnknownValueError:
		    print("could not understand audio")
		except sr.RequestError as e:
		    print("Recognition error; {0}".format(e))

	print(command)