import argparse
import sys
import string
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
from mobileTasks import *
from std_msgs.msg import String
from baxter_core_msgs.msg import CollisionDetectionState
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge
from gtts import gTTS 
import playsound

rawCommand = ""
collisionState = False

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
rospy.init_node('Test')
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

# amp up gripper holding force
lGripper.set_holding_force(100)
lGripper.set_moving_force(100)

rGripper.set_holding_force(100)
rGripper.set_moving_force(100)

lLimb.set_joint_position_speed(.5)
lLimb.set_command_timeout(2)
strtPose = lLimb.endpoint_pose()


slow = .02 # m/s
fast = .2 # m/s

command = ""
newCommand = ""
lastCommand = ""

# Event for pausing the current task
pause_event = Event()
#Move to Zero position()

def openFridge():
    global lLimb, rLimb, pause_event
    playPositionFile('open_fridge.wp', lLimb, rLimb, pause_event)

def closeFridge():
    global lLimb, rLimb, pause_event
    playPositionFile('close_fridge.wp', lLimb, rLimb, pause_event)

def getBottleOpenFridge():
    global lLimb, rLimb, pause_event, lGripper
    print("Getting bottle open fridge part 1")
    playPositionFile('get_bottle_open_fridge_p1.wp', lLimb, rLimb, pause_event)
    moveOnAxis(lLimb, 'y', .08, .06, pause_event)
    time.sleep(1)
    waitForNotPause(pause_event)
    lGripper.close()
    time.sleep(1)
    print("Getting bottle open fridge part 2")
    playPositionFile('get_bottle_open_fridge_p2.wp', lLimb, rLimb, pause_event)
    time.sleep(1)

def moveToTableAfterRetrieve():
    global lLimb, rLimb, pause_event, lGripper
    print("moving bottle to table p1")
    playPositionFile('move_bottle_to_table_p1.wp', lLimb, rLimb, pause_event)
    #moveOnAxis(lLimb, 'y', -.1, .04, pause_event)
   # moveOnAxis(lLimb, 'z', -.1, .02, pause_event)
    time.sleep(1)
    waitForNotPause(pause_event)
    lGripper.open()
    time.sleep(1)
   # moveOnAxis(lLimb, 'z', .08, .04, pause_event)
    moveOnAxis(lLimb, 'x', -0.5, .04, pause_event)
    playPositionFile('move_bottle_to_table_p2.wp', lLimb, rLimb, pause_event)


def openMicrowave():
	global lLimb, rLimb, pause_event, lGripper
	playPositionFile('open_microwave.wp', lLimb, rLimb, pause_event)

def closeMicrowave():
	global lLimb, rLimb, pause_event, lGripper
	playPositionFile('close_microwave.wp', lLimb, rLimb, pause_event)

def getFoodContainer():
	global lLimb, rLimb, lGripper, pause_event
	playPositionFile('get_food_from_fridge_p1.wp', lLimb, rLimb, pause_event)
	#moveOnAxis(lLimb, 'z', -.015, .03, pause_event)
	moveOnAxis(lLimb, 'y', .07, .03, pause_event)
	time.sleep(.5)
	waitForNotPause(pause_event)
	lGripper.close()
	time.sleep(.5)
	#moveOnAxis(lLimb, 'z', .03, .04, pause_event)
	moveOnAxis(lLimb, 'y', -.1, .03, pause_event)
	playPositionFile('get_food_from_fridge_p2.wp', lLimb, rLimb, pause_event)

def placeInMicrowave():
	global lLimb, rLimb, lGripper, pause_event
	playPositionFile('put_food_in_microwave_p1.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	lGripper.open()
	time.sleep(1)
	moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('putFoodInMicrowaveP2.wp', lLimb, rLimb, pause_event)

def turnOnMicrowave():
	global lLimb, rLimb, lGripper, pause_event
	playPositionFile('turn_on_microwave.wp', lLimb, rLimb, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'y', .07, .09, pause_event)
	time.sleep(.5)
	moveOnAxis(lLimb, 'z', -.10, .04, pause_event)	
	time.sleep(5)
	turnOffMicrowave()

def turnOffMicrowave():
	global lLimb, rLimb, lGripper, pause_event
	moveOnAxis(lLimb, 'z', .15, .07, pause_event)
	moveOnAxis(lLimb, 'z', -.03, .04, pause_event)
	moveOnAxis(lLimb, 'y', -.05, .05, pause_event)
	playPositionFile('turn_off_microwave.wp', lLimb, rLimb, pause_event)

def getFoodFromMicrowave():
	gripper.open()
	openMicrowave(lLimb, rLimb, pause_event)
	time.sleep(1)
	playPositionFile('get_food_from_microwave.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	gripper.close()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .02, .02, pause_event)
	moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('getFoodFromMicrowaveP2.wp', lLimb, rLimb, pause_event)
	gripper.open()
	playPositionFile('getFoodFromMicrowaveP3.wp', lLimb, rLimb, pause_event)
	moveOnAxis(lLimb, 'x', -.15, .05, pause_event)
	playPositionFile('getFoodFromMicrowaveP4.wp', lLimb, rLimb, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)

def getFoodFromOpenMicrowave():
	lGripper.open()
	time.sleep(1)
	playPositionFile('get_food_from_microwave_p1.wp', lLimb, rLimb, pause_event)
	time.sleep(1)
	moveOnAxis(lLimb, 'z', -.02, .02, pause_event)
	moveOnAxis(lLimb, 'y', .17, .02, pause_event)
	waitForNotPause(pause_event)
	time.sleep(1)
	lGripper.close()
	time.sleep(1)
	moveOnAxis(lLimb, 'z', .02, .02, pause_event)
	#moveOnAxis(lLimb, 'y', -.17, .02, pause_event)
	playPositionFile('get_food_from_microwave_p2.wp', lLimb, rLimb, pause_event)
	lGripper.open()
	moveOnAxis(lLimb, 'y', -.06, .02, pause_event)
	playPositionFile('get_food_from_microwave_p3.wp', lLimb, rLimb, pause_event)
	closeMicrowave(lLimb, rLimb, pause_event)
	moveToDownward(lLimb, rLimb, pause_event)


#print("Moving to mobile downward position")
#moveToDownward(lLimb, rLimb, pause_event)

#openMicrowave()
#getFoodFromOpenMicrowave()


playPositionFile('happyBirthday.wp', lLimb, rLimb, pause_event)
tts = gTTS(text='Happy Birthday Berry, Have a wonderful day. God bless you', lang='en')
tts.save("happy.mp3")
playsound.playsound("happy.mp3", True)
time.sleep(5)



#print("Opening the fridge")
'''
openFridge()
getBottleOpenFridge()
moveToTableAfterRetrieve()
closeFridge()

'''
'''


#print("Getting bottle in hand")
#getBottleOpenFridge()

#moveToTableAfterRetrieve()
#closeFridge()
openMicrowave()
closeMicrowave()

#openFridge()
#getFoodContainer()
#placeInMicrowave()
#turnOnMicrowave()
#turnOffMicrowave()

'''



