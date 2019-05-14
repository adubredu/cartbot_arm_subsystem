#!/usr/bin/env python

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

#Suggestion after timed cook!!
# booleans are wrong for microwave being on and off
# make boolean for where the food is
import argparse
import sys
import string
import time
from multiprocessing import Process
from threading import (
    Thread,
    activeCount,
    Event as TEvent,
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
from GUIhelper import *
#attempt to move task threading to new file -- :(
#from runTasks import *
from positionControl import *
from geometry_msgs.msg import Twist
from mobileTasks import *
from std_msgs.msg import String
from baxter_core_msgs.msg import CollisionDetectionState
from sensor_msgs.msg import Image
import time
from Tkinter import *
from gtts import gTTS
import playsound
from auto_park import auto_park
import subprocess, shlex


#Global variables
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5 )
rawCommand = ""
collisionState = False
lLimb = None
rLimb = None
lGripper = None
rGripper = None
t2s  = None
voices = None
r = None
m = None
goneforward = False
gonebackward = False
goneleft = False
goneright = False
range = 1
slow = .02 # m/s
fast = .2 # m/s
command = ""
newCommand = ""
lastCommand = ""
dialog = "Listening..."
# Environment Tracking Booleans
env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 'bottleInFridge': True, 
		'microwaveOpen': False, 'holdingSomething': False, 'microwaveOn': False, 'foodInMicrowave': False, 
		'foodInFridge': True, 'foodOnTable': False, 'robotlocalized': True, 'mobileBaseActivated': False})

# Event for pausing the current task
pause_event = TEvent()

#interface widgets#
#Dialog title
root = Tk()
root['bg'] = "#ccefff"
dialog_label = Label(root, text="Dialog", fg="black", bg="light blue", 
                     width=60, height=2, font=("Ariel", 20))
dialog_label.pack()
#Dialog box
dialog_box = Label(root, bg="white", padx=10, pady=10, 
                   borderwidth=1, relief=SOLID, anchor="nw",
                   width=100, height=4, font=("Ariel", 15),
                   wraplength=500, justify=LEFT)
dialog_content = StringVar()
dialog_content.set("Listening...")
dialog_box["textvariable"] = dialog_content
dialog_box.pack(fill = X)
#command label
command_box = Label(root, fg="black", bg="lightgrey",
                    bd=1, relief=SOLID, width=60, height=2, 
                    font=("Courier New", 19), wraplength=500)
curr_command = StringVar()
curr_command.set("Waiting...")
command_box["textvariable"] = curr_command
command_box.pack()
#blank label separator
#box
prompt = Frame(root, width=200, height=100, bg="#ccefff")
prompt.pack()
sug_header = Label(prompt, text="Suggested Commands", font=("Ariel",18), bg="#ccefff",
                    fg="#000033")
sug_header.pack(pady=10)
first_suggestion = StringVar()
first = Label(prompt, textvariable=first_suggestion, font = ("Ariel", 15), bg="#ccefff",
              fg="#181818")
first.pack(pady=5)
second_suggestion = StringVar()
second = Label(prompt, textvariable=second_suggestion, font = ("Ariel", 15), bg="#ccefff",
                fg="#181818")
second.pack(pady=2)

mode_value = StringVar()
mode_value.set("Robot not localized")
robot_mode = Label(root, textvariable=mode_value, fg="white", bg="red", 
	width=60, height=2, font=("Ariel", 20), bd=1, relief=SOLID)
robot_mode.pack()

#environment title
env_label = Label(root, text="Environment", fg="white", bg="#000d33",
                  width=60, height=2, font=("Ariel", 20), bd=1, relief=SOLID)
env_label.pack()

env_frame = Frame(root, width=200, height=100, bg="#ccefff")
env_frame.pack(pady=20)

fridge_value = StringVar()
fridge_value.set("Fridge: Closed")
fridge = Label(env_frame, textvariable=fridge_value, bg="light grey", bd=1, relief=SOLID, 
                width=20, height=6, font=("Ariel", 15))
fridge.pack(padx = 10, side=LEFT, pady=5)

microwave_value = StringVar()
microwave_value.set("Microwave: Off, Closed")
microwave = Label(env_frame, textvariable=microwave_value, bg="light grey", bd=1, 
                  relief=SOLID, width=20, height=6, font=("Ariel", 15))
microwave.pack(padx= 10, side=LEFT, pady=5)

env_frame2 = Frame(root, width=200, height=100, bg="#ccefff")
env_frame2.pack(pady=20)

bottle_value = StringVar()
bottle_value.set("Bottle: In fridge")
bottle = Label(env_frame2, textvariable=bottle_value, width=20, height=6, bd=1, relief=SOLID,
                font=("Ariel", 15), bg="light grey")
bottle.pack(padx= 10, side=LEFT, pady=5) 

food_value = StringVar()
food_value.set("food: In fridge")
food = Label(env_frame2, textvariable=food_value, width=20, height=6, bd=1, relief=SOLID,
                font=("Ariel", 15), bg="light grey")
food.pack(padx= 10, side=LEFT, pady=5) 

#environment tokens#
at_start = True;

def get_prompt():
    global at_start, lastAliveName, env
    if (at_start):
        make_label('"Robot is localized"', '"Activate auto localization"')
        at_start = False
    else:
        suggestions = make_suggestion(lastAliveName, env)
       	make_label(suggestions[0], suggestions[1])

def make_label(sug1, sug2):
        if (sug1 != ""):
                first_suggestion.set(sug1)
        else:
                first_suggestion.set("No suggested commands at this time")
                second_suggestion.set("")
                return
        if (sug2 != ""):
                second_suggestion.set(sug2)
        else:
                second_suggestion.set("")

# end interface widgets #
def update_command_box(command):
    #transform to real string
    command_string = command_to_string(command)
    curr_command.set(command_string)
    command_box["bg"] = "green"



# Helper functions #
def terminate_thread(thread):
    if not thread.isAlive():
        return
    exc = ctypes.py_object(KeyboardInterrupt)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
    ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

def collisionDetection(data):
    global collisionState
    global rawCommand
    collisionState = data.collision_state
    if collisionState:
        rawCommand = ""

# clamping function to constrain arm movement
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
# Definitions #


#for launching mobile base scripts to activate mobile base
def activate_mobile_base():
    command = "gnome-terminal --tab -e './run_mobile_base.sh'"
    args = shlex.split(command)
    p1 = subprocess.call(args)

def activate_auto_pilot():
    command = "gnome-terminal --tab -e './auto_park.py'"
    args = shlex.split(command)
    p1 = subprocess.call(args)

def setup_baxter():
    print "in baxter setup"
    rospy.init_node('speechControl')
    global lLimb, rLimb, lGripper, rGripper
    lLimb = baxter.Limb('left')
    rLimb = baxter.Limb('right')
    lGripper = baxter.Gripper('left')
    rGripper = baxter.Gripper('right')
    baxter_enabler = baxter.RobotEnable(versioned=True)
    baxter_enabler.enable()
    # Set up subscriber for collision detection
    name = '/robot/limb/left/collision_detection_state'
    collisionSubs = rospy.Subscriber(name=name, 
                                     data_class=CollisionDetectionState, 
                                     callback=collisionDetection, buff_size=100)
    if not lGripper.calibrate():
        print("left gripper did not calibrate")
        sys.exit()
    lGripper.set_holding_force(100)
    lGripper.set_moving_force(100)
    rGripper.set_holding_force(100)
    rGripper.set_moving_force(100)
    lLimb.set_joint_position_speed(.5)
    lLimb.set_command_timeout(2)
    strtPose = lLimb.endpoint_pose()

def setup_speech():
    print "in speech setup"
    global m, r
    r = sr.Recognizer()
    m = sr.Microphone()
    r.pause_threshold = .5
    r.dynamic_energy_threshold = False
    with m as source:
        r.adjust_for_ambient_noise(source, 1)

# callback function when audio data is obtained
def heard(recognizer, audio):
    # Defining Commands to be accepted
    global t2s, dialog
    #credsJson = ""
    #with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
        #credsJson = gspeechcreds.read()
    
    sens = 1
    commands = ["move arm forward", "move arm backward", "move arm left", "move arm right", "move arm up", "move arm down", 
                "move arm higher", "move arm lower", "close hand", "open hand", "stop", "stop", "stop", "open the microwave",
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

def runTask(task_target, task_name, args):
    global task, lLimb, rLimb, pause_event, rawCommand
    task = Thread(target=task_target, args=args,
                  name=task_name)
    print(task.name)
    pause_event.clear()
    task.daemon = True
    task.start()
    update_command_box(task_name)
    rawCommand = ""


def update_after_task():
    global t2s, lastAliveName
    #tts = gTTS(text='Done with '+ lastAliveName, lang='en')
    #tts.save("donewith.mp3")
    #playsound.playsound('donewith.mp3', True)
    
    curr_command.set("Waiting...")
    command_box['bg'] = "light grey"

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
        env['bottleOnTable'] = True
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
        env['bottleOnTable'] = True
    elif lastAliveName == "gettingBottleFromOpenFridge":
        env['fridgeOpen'] = False
        env['hasBottle'] = False
    elif lastAliveName == "puttingFoodInMicrowave":
        env['foodInMicrowave'] = True
        env['foodInFridge'] = False
        env['fridgeOpen'] = False
        env['microwaveOpen'] = False
    elif lastAliveName == "gettingFoodFromMicrowave":
        env['foodInMicrowave'] = False
        env['microwaveOpen'] = False
        env['holdingSomething'] = False
        env['foodOnTable'] = True
    elif lastAliveName == "timedCook":
        env['foodInMicrowave'] = True
        env['microwaveOpen'] = False
        env['microwaveOn'] = False
    elif lastAliveName == "activateAutoParking":
            env['robotlocalized'] = True
    elif lastAliveName == "activateMobileBase":
        env['mobileBaseActivated'] = True
        env['robotlocalized'] = False
    update_tokens()

def update_tokens():
	#get_prompt()
	update_bottle()
	update_fridge()
	update_microwave()
	update_food()  

def update_food():
    if env['foodInFridge']:
    	food_value.set("Food: in Fridge")
    elif env['foodInMicrowave']:
    	food_value.set("Food: in Microwave")
    	food['bg'] = 'light green'
    elif env['foodOnTable']:
    	food_value.set("Food: On Table")

def update_bottle():
    if (env['hasBottle']):
        bottle_value.set("Bottle: In hand")
        bottle['bg'] = 'light green'
    if (env['bottleOnTable']):
        bottle_value.set("Bottle: On table")
        bottle['bg'] = 'light grey'

def update_fridge():
    if(env['fridgeOpen']):
        fridge_value.set("Fridge: Open")
        fridge['bg'] = "light green"
    if(not env['fridgeOpen']):
        fridge_value.set("Fridge: Closed")
        fridge['bg'] = "light grey"

def update_microwave():
    if(env['microwaveOn'] and env['microwaveOpen']):
        microwave_value.set("Microwave: On, Open")
        microwave['bg'] = "light green"
    elif (env['microwaveOpen'] and not env['microwaveOn']):
        microwave_value.set("Microwave: Off, Open")
        microwave['bg'] = "light green"
    elif (not env['microwaveOpen'] and env['microwaveOn']):
        microwave_value.set("Microwave: On, Closed")
        microwave['bg'] = "light green"
    elif (not env['microwaveOpen'] and not env['microwaveOn']):
        microwave_value.set("Microwave: Off, Closed")
        microwave['bg'] = "light grey"


root.title("Baxter Kitchen Helper")
root.minsize(width=800, height=1000)
root.maxsize(width=800, height=1000)
setup_baxter()
setup_speech()

stopListening = r.listen_in_background(m, heard, phrase_time_limit=4)

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

get_prompt()

while not rospy.is_shutdown():

    #update the GUI
    root.update()
    dialog_content.set(dialog)

    # Check if task has ended
    if lastAliveStatus and not task.is_alive():
        update_after_task()
        #set_tokens()

    lastAliveStatus = task.is_alive()
    lastAliveName = task.name
    newCommand = rawCommand

    # Check if command has changed
    if command != newCommand:
        if newCommand != "": 
            speed = slow
        command = newCommand
        orient = lLimb.endpoint_pose()['orientation']
        #print("last: {}, this: {}".format(lastCommand, command))

    if collisionState:
        print "Can't Move Here"
        if task and task.is_alive():
            rawCommand = "stop"
    


    ##### Execute relevant command #####
    #### Directional Commands #####
    if 'move arm forward' in command:
        terminate_thread(task)
        args=(lLimb, 'y', 4, speed, pause_event)
        runTask(moveOnAxis, "movingForward", args)
    if 'move arm backward' in command:
        terminate_thread(task)
        args=(lLimb, 'y', -4, speed, pause_event)
        runTask(moveOnAxis, "movingBackward", args)
    if 'move arm left' in command:
        terminate_thread(task)
        args=(lLimb, 'x', -4, speed, pause_event)
        runTask(moveOnAxis, "movingLeft", args)
    if 'move arm right' in command:
        terminate_thread(task)
        args=(lLimb, 'x', 4, speed, pause_event)
        runTask(moveOnAxis, "movingRight", args)
    if  'arm' in command and ('higher' in command or 'up' in command):
        terminate_thread(task)
        args=(lLimb, 'z', 4, speed, pause_event)
        runTask(moveOnAxis, "movingHigher", args)
    if  'arm' in command and ('lower' in command or 'down' in command):
        terminate_thread(task)
        args=(lLimb, 'z', -4, speed, pause_event)
        runTask(moveOnAxis, "movingLower", args)
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
        curr_command.set('Stopped')
        command_box['bg'] = "red"
    if 'continue' in command:
        rawCommand = ""
        curr_command.set(lastAliveName)
        command_box["bg"] = "green"
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

    if 'ground grab mode' in command:
    	terminate_thread(task)
    	args=(lLimb, rLimb, lGripper, pause_event)
    	runTask(groundPickUpMode, "Ground pick up mode", args)

    if 'table grab mode' in command:
    	terminate_thread(task)
    	args=(lLimb, rLimb, lGripper, pause_event)
    	runTask(tablePickUpMode, "Table pick up mode", args)
    
    if ('pick' in command and 'object' in command) or 'floor' in command:
        terminate_thread(task)
        args = (lLimb, rLimb,lGripper,  pause_event)
        runTask(pickFromFloor, "pickFromFloor", args)

    if ('drop' in command and 'object' in command):
	terminate_thread(task)
	args=(lLimb, rLimb, lGripper, pause_event)
	runTask(dropObject, "dropObject", args)

    if ('put' in command.lower() and 'table' in command.lower()):
	terminate_thread(task)
	args=(lLimb, rLimb, lGripper, pause_event)
	runTask(putOnTable, "putOnTable", args)
    
    if 'forward' in command and 'arm' not in command:
        velocity = Twist()
        velocity.linear.x = 1.0
        vel_pub.publish(velocity)
        time.sleep(1)
        v=Twist()
        vel_pub.publish(v)

    if 'backward' in command and 'arm' not in command:
        velocity = Twist()
        velocity.linear.x = -1.0
        vel_pub.publish(velocity)
        time.sleep(1)
        v=Twist()
        vel_pub.publish(v)


    if 'left' in command and 'arm' not in command:
        velocity = Twist()
        velocity.angular.z = 1.0
        vel_pub.publish(velocity)
        time.sleep(1)
        v=Twist()
        vel_pub.publish(v)

    if 'right' in command and 'arm' not in command:
        velocity = Twist()
        velocity.angular.z = -1.0
        vel_pub.publish(velocity)
        time.sleep(1)
        v=Twist()
        vel_pub.publish(v)

    if 'pause' in command:
        velocity = Twist()
        vel_pub.publish(velocity)
        time.sleep(1)
        time.sleep(1)
        v=Twist()
        vel_pub.publish(v)
	
    if 'long range' in command:
	range = 3

    if 'short range' in command:
	range = 1
    ### Begin task commands ###

    # NOTE!
    # The fridge and microwave tasks should only run if the environment variable
    # env['robotlocalized'] is True. 
    # Else they won't be executed

    if env['robotlocalized']:
	        ### Begin fridge commands ### 
	        # Open the fridge #
	    if 'open' in command and 'the fridge' in command:
	        terminate_thread(task)
	        if not env['fridgeOpen']:
	            args=(lLimb, rLimb, pause_event)
	            runTask(openFridge, "openingFridge", args)
	    # Get waterbottle from fridge #
	    if 'get' in command and 'water bottle' in command:
	        terminate_thread(task)
	        # if fridge is closed #
	        if not env['fridgeOpen'] and not env['hasBottle']:
	            args = (lLimb, rLimb,lGripper, pause_event)
	            runTask(getBottleFromStart, "gettingBottleFromStart", args)
	        # if fridge is open #
	        if env['fridgeOpen'] and not env['hasBottle']:
	            args=(lLimb, rLimb,lGripper, pause_event)
	            runTask(pickBottleFromOpenFridge, "gettingBottleFromOpenFridge", 
	                    args)
	        # if fridge is open and has bottle #
	        if env['fridgeOpen'] and env['hasBottle']:
	            rawCommand = ""
	            pass
	    # Close the fridge #
	    if 'close' in command and 'the fridge' in command:
	        terminate_thread(task)
	        if env['fridgeOpen'] and not env['holdingSomething']:
	            args=(lLimb, rLimb, pause_event)
	            runTask(closeFridge, "closingFridge", args)     
	    # Place water bottle on table #
	    if 'place' in command and 'table' in command:
	        terminate_thread(task)
	        if env['hasBottle']:
	            args=(lLimb, rLimb,lGripper, pause_event)
	            runTask(moveToTableAfterRetrieve, "placingOnTable", args)
	        else:
	            print("I don't have the bottle right now")
	            print(task.name)
	            rawCommand = ""
	    # Full command - Put water bottle on table #
	    if 'put' in command and 'water bottle' in command and 'table' in command:
	        terminate_thread(task)
	        args=(lLimb, rLimb,lGripper,pause_event)
	        # if the fridge is closed #
	        if not env['fridgeOpen'] and not env['hasBottle']:
	            runTask(getBottleFull, "gettingBottlePlacingOnTable", args)
	        # if the fridge is open #
	        elif env['fridgeOpen'] and not env['hasBottle']:
	            runTask(bottleOnTableAfterOpenFridge, 
	                    "gettingBottleFromOpenFridgeAndPlacingOnTable", args)
	        # if it has water bottle already #
	        elif env['fridgeOpen'] and env['hasBottle']:
	            runTask(moveToTableAfterRetrieve, "placingOnTable", args)
	    #### End Fridge task commands ####

	    # Water Bottle Commands #
	    ## Stub for now

	    ### Microwave commands ###
	    if 'open' in command and 'the microwave' in command:
	        terminate_thread(task)
	        if not env["microwaveOpen"] and not env["holdingSomething"]:
	            args=(lLimb, rLimb, pause_event)
	            runTask(openMicrowave, "openingMicrowave", args)
	        rawCommand = ""
	    if 'close' in command and 'microwave' in command:
	        terminate_thread(task)
	        if env["microwaveOpen"] and not env["holdingSomething"]:
	            args=(lLimb, rLimb, pause_event)
	            runTask(closeMicrowave, "closingMicrowave", args)
	        rawCommand = ""
	    if 'start' in command and 'microwave' in command:
	        terminate_thread(task)
	        if (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
	            args=(lLimb, rLimb, pause_event)
	            runTask(turnOnMicrowave, "turningOnMicrowave", args)
	            env["microwaveOn"] = True
	        rawCommand = ""
	    if ('turn off' in command) or ('stop'in command and 'microwave' in command):
	        terminate_thread(task)
	        if env["microwaveOn"] and (not env["holdingSomething"]):
	            args=(lLimb, rLimb, pause_event)
	            runTask(turnOffMicrowave, "turningOffMicrowave", args)
	        rawCommand = ""
	    if 'cook' in command and 'seconds' in command:
	        terminate_thread(task)
	        t = [int(s) for s in command.split() if s.isdigit()]
	        print(t)
	        if not t:
	            print("No time given")
	        elif (not env["microwaveOpen"]) and (not env["holdingSomething"]) and (not env["microwaveOn"]):
	            args=(lLimb, rLimb, pause_event, t[0],)
	            runTask(timedMicrowave, "timedCook", args)
	            env["microwaveOn"] = True
	        rawCommand = ""
	    if (('meal' in command) or ('food' in command)) and (('put' in command or 'place' in command) and ('microwave' in command)):
	        terminate_thread(task)
	        if not env["fridgeOpen"] and not env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
	            args=(lLimb, rLimb, lGripper, pause_event,)
	            runTask(placeContainerInMicrowaveFromStart, "puttingFoodInMicrowave"
	                    ,args)
	        if env["fridgeOpen"] and not env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
	            args=(lLimb, rLimb, lGripper, pause_event,)
	            runTask(placeContainerInMicrowaveFromOpenFridge, 
	                    "puttingFoodInMicrowave", args)
	        if not env["fridgeOpen"] and env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
	            args=(lLimb, rLimb, lGripper, pause_event,)
	            runTask(placeContainerInMicrowaveFromOpenMicrowave, 
	                "puttingFoodInMicrowave", args)
	        if env["fridgeOpen"] and env["microwaveOpen"] and not env["holdingSomething"] and not env["foodInMicrowave"]:
	            args=(lLimb, rLimb, lGripper, pause_event,)
	            runTask(placeContainerInMicrowaveFromOpenMicOpenFridge,
	                "puttingFoodInMicrowave", args) 
	        rawCommand = ""
	    if ('take' in command or 'get' in command) and ('container' in command or 'food' in command) and ('microwave' in command):
	        terminate_thread(task)
	        if not env["microwaveOpen"] and env["foodInMicrowave"]:
	            args=(lLimb, rLimb, lGripper, pause_event,)
	            runTask(getFoodFromMicrowave, "gettingFoodFromMicrowave", args)
	        if env["microwaveOpen"] and env["foodInMicrowave"]:
	            args=(lLimb, rLimb, lGripper, pause_event,)
	            runTask(getFoodFromOpenMicrowave, "gettingFoodFromMicrowave", args)
	        rawCommand = ""
	    # end microwave Commands
	    ### End task related commands ###
    
#
#
#
#

    # CONSIDER THE ELIF'S WHEN DEBUGGING UI
#
#
#
#
    ### environment variable control commands ###           
    if 'fridge is open' in command:
        env['fridgeOpen'] = True
        update_fridge()
    elif "fridge is closed" in command:
        env['fridgeOpen'] = False
        update_fridge()
    elif "holding something" in command:
        env['hasBottle'] = True
        env['holdingSomething'] = True
        update_bottle()
    elif "hand is empty" in command:
        lGripper.open()
        env['hasBottle'] = False
        env['holdingSomething'] = True
        update_bottle()
    elif "microwave is open" in command:
        env['microwaveOpen'] = True
        update_microwave()
    elif "microwave is closed" in command:
        env['microwaveOpen'] = False
        update_microwave()
    elif 'microwave is on' in command:
        env['microwaveOn'] = True
        update_microwave()
    elif 'microwave is off' in command:
        env['microwaveOn'] = False 
        update_microwave()
    elif 'food in microwave' in command:
        env['foodInMicrowave'] = True
        env['foodInFridge'] = False
        env['foodOnTable'] = False
        update_food()
    elif 'microwave is empty' in command:
        env['foodInMicrowave'] = False
        update_microwave()
    elif 'robot is localized' in command:
        env['robotlocalized'] = True
        mode_value.set("Robot Localized")
        robot_mode['bg'] = 'light green'
    elif 'food in fridge' in command:
    	env['foodInFridge'] = True
    	update_food()
    elif 'food on table' in command:
    	env['foodOnTable'] = True
    	env['foodInFridge'] = False
    	env['foodInMicrowave'] = False
    	update_food()

    #update_tokens()
    

    ### end environment variable control commands

    ### miscellaneous commands ###
    if 'move to zero' in command:
        terminate_thread(task)
        args = (lLimb, rLimb, pause_event)
        runTask(moveToDownward, "movingToZero", args)
    if 'test' in command:
        terminate_thread(task)
        task = Thread(target=tester, args=(lLimb,pause_event))
        print(task.name)
        pause_event.clear()
        task.start()
        rawCommand = ""
    if 'reset environment' in command:
        env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 'bottleInFridge':True, 
                'microwaveOpen': False, 'holdingSomething': False, 
                'microwaveOn': False, 'foodInMicrowave': False, 'foodOnTable':False, 'mobileBaseActivated':False, 'foodInFridge':True,'robotlocalized':True})
        rawCommand = ""

    ### mobile base commands
    if 'localization' in command:
        terminate_thread(task)
        task = Thread(target=activate_auto_pilot, name="activateAutoParking")
        print(task.name)
        pause_event.clear()
        task.daemon = True
        task.start()
        rawCommand = ""

    if 'mobile' in command:
        terminate_thread(task)
        task = Thread(target=activate_mobile_base, name="activateMobileBase")
        print(task.name)
        pause_event.clear()
        task.daemon = True
        task.start()
        rawCommand = ""
    ### End Miscellaneous commands ###
    

    newCommand = ""
    lastCommand = command
    time.sleep(.01)
# end command while loop #

# Clean Up on end
terminate_thread(task)
if env['hasBottle']:
    pause_event.clear
    rospy.init_node('ending_node')
    moveToTableAfterRetrieve(lLimb, rLimb, lGripper, pause_event)
stopListening()
print('ending')
